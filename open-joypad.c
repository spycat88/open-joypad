// SPDX-License-Identifier: GPL-2.0
/*
 * Open Joypad Driver
 *
 * Supports:
 *   1) No ADC mode (GPIO-only)
 *   2) Multi-ADC mode (multiple iio channels)
 *   3) Single-ADC + analog multiplexer mode
 *   4) Miyoo serial mode
 *
 * The device tree property "joypad-mode" must be one of:
 *   - "none"        -> No ADC, only GPIO
 *   - "multiadc"    -> Multi-ADC approach
 *   - "singleadc"   -> Single-ADC with analog multiplexer
 *   - "miyooserial" -> Miyoo serial-based joystick input
 *
 * Example usage in device tree is provided at the end of this file.
 */

#include <linux/module.h>
#include <linux/input-polldev.h>
#include <linux/platform_device.h>
#include <linux/iio/consumer.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0))
#include <linux/of_gpio.h>
#else
#include <linux/of_gpio_legacy.h>
#endif
#include <linux/pwm.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/fs.h>
#include <linux/termios.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>

/* Constants for IIO channels and typical voltage scaling. */
#define SARADC_CH_NUM  8
#define ADC_MAX_VOLTAGE  1800
#define ADC_TUNING_DEFAULT  180

#define DRV_NAME  "open-joypad"

/* Macros to clamp and do percentage-based tuning. */
#define CLAMP(x, low, high) \
	(((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define ADC_DATA_TUNING(x, p)  ((x * p) / 100)

/* Miyoo-specific definitions. */
#define MIYOO_FRAME_SIZE            6
#define MIYOO_MAGIC_START           0xFF
#define MIYOO_MAGIC_END             0xFE
#define MIYOO_AXIS_MIN              (-32760)
#define MIYOO_AXIS_MAX              ( 32760)
#define MIYOO_CALIBRATION_FRAMES    50
#define MIYOO_SERIAL_INACTIVITY_TIMEOUT  (5 * HZ)

/* Driver operating modes */
enum joypad_mode {
	JOYPAD_MODE_NONE = 0,
	JOYPAD_MODE_MULTIADC,
	JOYPAD_MODE_SINGLEADC,
	JOYPAD_MODE_MIYOOSERIAL,
};

/* Per-channel config for multi-ADC mode. */
struct bt_adc {
	int value;
	int report_type;
	int max;
	int min;
	int cal;
	int scale;
	bool invert;
	int channel;
	int tuning_p;
	int tuning_n;
};

/* Per-button GPIO config. */
struct bt_gpio {
	const char *label;
	int num;
	int report_type;
	int linux_code;
	bool old_value;
	bool active_level;
};

/* Single-ADC approach: analog multiplexer lines. */
struct analog_mux {
	struct iio_channel *iio_ch;
	int sel_a_gpio;
	int sel_b_gpio;
	int en_gpio;
};

/* Per-channel config for single-ADC approach. */
struct bt_adc_single {
	int value;
	int report_type;
	int max;
	int min;
	int cal;
	int scale;
	bool invert;
	int amux_ch;
	int tuning_p;
	int tuning_n;
};

/* Miyoo joystick calibration info. */
struct miyoo_cal {
	int x_min;
	int x_max;
	int x_zero;
	int y_min;
	int y_max;
	int y_zero;
};

/* Miyoo two-stick container. */
struct miyoo_serial_stick {
	struct miyoo_cal left_cal;
	struct miyoo_cal right_cal;
	int left_x;
	int left_y;
	int right_x;
	int right_y;
};

/* Main driver private data. */
struct joypad {
	struct device *dev;
	enum joypad_mode mode;
	int poll_interval;
	bool enable;

	/* GPIO buttons */
	int bt_gpio_count;
	struct bt_gpio *gpios;
	bool auto_repeat;

	/* ADC / analog input common settings */
	int bt_adc_fuzz;
	int bt_adc_flat;
	int bt_adc_scale;
	int bt_adc_deadzone;

	/* Inversion flags */
	bool invert_absx;
	bool invert_absy;
	bool invert_absz;
	bool invert_absrx;
	bool invert_absry;
	bool invert_absrz;

	struct mutex lock;
	struct input_polled_dev *poll_dev;
	struct input_dev *input;

	/* Multi-ADC fields */
	struct iio_channel *adc_ch[SARADC_CH_NUM];
	int chan_count;
	struct bt_adc *adcs;

	/* Single-ADC multiplexer fields */
	struct analog_mux *amux;
	int amux_count;
	struct bt_adc_single *adcs_single;

	/* Rumble (PWM) fields */
	struct pwm_device *pwm;
	struct work_struct play_work;
	u16 level;
	u16 boost_weak;
	u16 boost_strong;
	bool has_rumble;
	bool rumble_enabled;

	/* Miyoo serial fields */
	struct miyoo_serial_stick miyoo;
	struct task_struct *miyoo_thread;
	struct file *miyoo_filp;
	struct delayed_work miyoo_init_work;
	unsigned long last_data_jiffies;
};

/* Global reference if needed externally. */
extern struct input_dev *joypad_input_g;
struct input_dev *joypad_input_g;

/* -------------------------------------------------------------------------- */
/*                           Rumble (PWM) Support                             */
/* -------------------------------------------------------------------------- */

static int pwm_vibrator_start(struct joypad *joypad)
{
	struct pwm_state state;
	int err;

	pwm_get_state(joypad->pwm, &state);
	pwm_set_relative_duty_cycle(&state, joypad->level, 0xffff);
	state.enabled = true;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0))
	err = pwm_apply_state(joypad->pwm, &state);
#else
	err = pwm_apply_might_sleep(joypad->pwm, &state);
#endif

	if (err)
		dev_err(joypad->dev, "failed to apply PWM state: %d\n", err);

	return err;
}

static void pwm_vibrator_stop(struct joypad *joypad)
{
	pwm_disable(joypad->pwm);
}

static void pwm_vibrator_play_work(struct work_struct *work)
{
	struct joypad *joypad = container_of(work, struct joypad, play_work);

	mutex_lock(&joypad->lock);

	if (!joypad->rumble_enabled) {
		pwm_vibrator_stop(joypad);
		mutex_unlock(&joypad->lock);
		return;
	}

	if (joypad->level)
		pwm_vibrator_start(joypad);
	else
		pwm_vibrator_stop(joypad);

	mutex_unlock(&joypad->lock);
}

/* Force-Feedback (rumble) callback */
static int rumble_play_effect(struct input_dev *dev,
			      void *data, struct ff_effect *effect)
{
	struct joypad *joypad = data;
	u32 boosted_level;

	if (effect->type != FF_RUMBLE)
		return 0;

	mutex_lock(&joypad->lock);

	if (!joypad->rumble_enabled) {
		mutex_unlock(&joypad->lock);
		return 0;
	}

	if (effect->u.rumble.strong_magnitude)
		boosted_level = effect->u.rumble.strong_magnitude + joypad->boost_strong;
	else
		boosted_level = effect->u.rumble.weak_magnitude + joypad->boost_weak;

	joypad->level = (u16)CLAMP(boosted_level, 0, 0xffff);

	mutex_unlock(&joypad->lock);

	schedule_work(&joypad->play_work);
	return 0;
}

/* Sysfs toggling for rumble */
static ssize_t joypad_store_rumble_enable(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);
	bool enable = simple_strtoul(buf, NULL, 10);

	mutex_lock(&joypad->lock);

	if (enable && !joypad->rumble_enabled) {
		joypad->rumble_enabled = true;
	} else if (!enable && joypad->rumble_enabled) {
		joypad->rumble_enabled = false;
		joypad->level = 0;
		cancel_work_sync(&joypad->play_work);
		pwm_vibrator_stop(joypad);
	}

	mutex_unlock(&joypad->lock);
	return count;
}

static ssize_t joypad_show_rumble_enable(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", joypad->rumble_enabled ? 1 : 0);
}

static DEVICE_ATTR(rumble_enable, S_IWUSR | S_IRUGO,
		   joypad_show_rumble_enable,
		   joypad_store_rumble_enable);

static struct attribute *joypad_rumble_attrs[] = {
	&dev_attr_rumble_enable.attr,
	NULL,
};

static struct attribute_group joypad_rumble_attr_group = {
	.attrs = joypad_rumble_attrs,
};

/* -------------------------------------------------------------------------- */
/*                         GPIO (Digital) Input Checks                        */
/* -------------------------------------------------------------------------- */

static void joypad_gpio_check(struct joypad *joypad)
{
	int i;

	for (i = 0; i < joypad->bt_gpio_count; i++) {
		struct bt_gpio *gpio = &joypad->gpios[i];
		int value;

		if (gpio_get_value_cansleep(gpio->num) < 0) {
			dev_err(joypad->dev, "failed to get gpio state\n");
			continue;
		}
		value = gpio_get_value(gpio->num);

		if (value != gpio->old_value) {
			input_event(joypad->poll_dev->input,
				    gpio->report_type,
				    gpio->linux_code,
				    (value == gpio->active_level) ? 1 : 0);
			gpio->old_value = value;
		}
	}

	input_sync(joypad->poll_dev->input);
}

/* -------------------------------------------------------------------------- */
/*                        Multi-ADC Reading & Handling                        */
/* -------------------------------------------------------------------------- */

static int joypad_adc_read_multi(struct joypad *joypad, struct bt_adc *adc)
{
	int value;

	if (iio_read_channel_processed(joypad->adc_ch[adc->channel], &value) < 0)
		return 0;

	value *= adc->scale;
	return value;
}

static void joypad_adc_check_multiadc(struct input_polled_dev *poll_dev)
{
	struct joypad *joypad = poll_dev->private;
	int i;

	/* We assume each pair is (X, Y) or (RX, RY), etc. */
	for (i = 0; i < joypad->chan_count; i += 2) {
		int mag;
		struct bt_adc *adcx = &joypad->adcs[i];
		struct bt_adc *adcy = &joypad->adcs[i + 1];

		adcx->value = joypad_adc_read_multi(joypad, adcx);
		adcy->value = joypad_adc_read_multi(joypad, adcy);

		adcx->value -= adcx->cal;
		adcy->value -= adcy->cal;

		/* Apply radial deadzone if needed. */
		mag = int_sqrt((adcx->value * (long)adcx->value)
			+ (adcy->value * (long)adcy->value));

		if (joypad->bt_adc_deadzone) {
			if (mag <= joypad->bt_adc_deadzone) {
				adcx->value = 0;
				adcy->value = 0;
			} else {
				adcx->value = (((adcx->max * adcx->value) / mag)
					* (mag - joypad->bt_adc_deadzone))
					/ (adcx->max - joypad->bt_adc_deadzone);

				adcy->value = (((adcy->max * adcy->value) / mag)
					* (mag - joypad->bt_adc_deadzone))
					/ (adcy->max - joypad->bt_adc_deadzone);
			}
		}

		/* Per-axis tuning (positive/negative) */
		if (adcx->tuning_n && adcx->value < 0)
			adcx->value = ADC_DATA_TUNING(adcx->value, adcx->tuning_n);
		if (adcx->tuning_p && adcx->value > 0)
			adcx->value = ADC_DATA_TUNING(adcx->value, adcx->tuning_p);

		if (adcy->tuning_n && adcy->value < 0)
			adcy->value = ADC_DATA_TUNING(adcy->value, adcy->tuning_n);
		if (adcy->tuning_p && adcy->value > 0)
			adcy->value = ADC_DATA_TUNING(adcy->value, adcy->tuning_p);

		/* Clamp and possibly invert. */
		adcx->value = CLAMP(adcx->value, adcx->min, adcx->max);
		adcy->value = CLAMP(adcy->value, adcy->min, adcy->max);

		input_report_abs(poll_dev->input, adcx->report_type,
			adcx->invert ? -adcx->value : adcx->value);
		input_report_abs(poll_dev->input, adcy->report_type,
			adcy->invert ? -adcy->value : adcy->value);
	}

	input_sync(poll_dev->input);
}

/* -------------------------------------------------------------------------- */
/*                     Single-ADC (Multiplexer) Handling                      */
/* -------------------------------------------------------------------------- */

static int joypad_amux_select(struct analog_mux *amux, int channel)
{
	/* This function sets SEL_A / SEL_B lines, and optionally EN. */
	gpio_set_value(amux->en_gpio, 0);

	switch (channel) {
	case 0:
		gpio_set_value(amux->sel_a_gpio, 0);
		gpio_set_value(amux->sel_b_gpio, 0);
		break;
	case 1:
		gpio_set_value(amux->sel_a_gpio, 0);
		gpio_set_value(amux->sel_b_gpio, 1);
		break;
	case 2:
		gpio_set_value(amux->sel_a_gpio, 1);
		gpio_set_value(amux->sel_b_gpio, 0);
		break;
	case 3:
		gpio_set_value(amux->sel_a_gpio, 1);
		gpio_set_value(amux->sel_b_gpio, 1);
		break;
	default:
		/* Disable if an invalid channel is selected. */
		gpio_set_value(amux->en_gpio, 1);
		return -EINVAL;
	}

	usleep_range(10, 20);
	return 0;
}

static int joypad_adc_read_single(struct analog_mux *amux, struct bt_adc_single *adc)
{
	int value, ret;

	ret = joypad_amux_select(amux, adc->amux_ch);
	if (ret)
		return 0;

	/* For single ADC, we read raw. Optionally .processed could be used. */
	iio_read_channel_raw(amux->iio_ch, &value);
	value *= adc->scale;

	return value;
}

static void joypad_adc_check_singleadc(struct input_polled_dev *poll_dev)
{
	struct joypad *joypad = poll_dev->private;
	int i;

	/* Similarly, we assume pairs of channels (X,Y) or (RY,RX)... */
	for (i = 0; i < joypad->amux_count; i += 2) {
		int mag;
		struct bt_adc_single *adcx = &joypad->adcs_single[i];
		struct bt_adc_single *adcy = &joypad->adcs_single[i + 1];

		adcx->value = joypad_adc_read_single(joypad->amux, adcx) - adcx->cal;
		adcy->value = joypad_adc_read_single(joypad->amux, adcy) - adcy->cal;

		mag = int_sqrt((adcx->value * (long)adcx->value)
			+ (adcy->value * (long)adcy->value));

		if (joypad->bt_adc_deadzone) {
			if (mag <= joypad->bt_adc_deadzone) {
				adcx->value = 0;
				adcy->value = 0;
			} else {
				adcx->value = (((adcx->max * adcx->value) / mag)
					* (mag - joypad->bt_adc_deadzone))
					/ (adcx->max - joypad->bt_adc_deadzone);

				adcy->value = (((adcy->max * adcy->value) / mag)
					* (mag - joypad->bt_adc_deadzone))
					/ (adcy->max - joypad->bt_adc_deadzone);
			}
		}

		/* Per-axis tuning (positive/negative) */
		if (adcx->tuning_n && adcx->value < 0)
			adcx->value = ADC_DATA_TUNING(adcx->value, adcx->tuning_n);
		if (adcx->tuning_p && adcx->value > 0)
			adcx->value = ADC_DATA_TUNING(adcx->value, adcx->tuning_p);

		if (adcy->tuning_n && adcy->value < 0)
			adcy->value = ADC_DATA_TUNING(adcy->value, adcy->tuning_n);
		if (adcy->tuning_p && adcy->value > 0)
			adcy->value = ADC_DATA_TUNING(adcy->value, adcy->tuning_p);

		adcx->value = CLAMP(adcx->value, adcx->min, adcx->max);
		adcy->value = CLAMP(adcy->value, adcy->min, adcy->max);

		input_report_abs(poll_dev->input, adcx->report_type,
				 adcx->invert ? -adcx->value : adcx->value);
		input_report_abs(poll_dev->input, adcy->report_type,
				 adcy->invert ? -adcy->value : adcy->value);
	}

	input_sync(poll_dev->input);
}

/* -------------------------------------------------------------------------- */
/*                       Miyoo Serial Thread & Helpers                        */
/* -------------------------------------------------------------------------- */

static int miyoo_calibrate_axis_x(const struct miyoo_cal *cal, int raw)
{
	int rangePos = cal->x_max - cal->x_zero;
	int rangeNeg = cal->x_zero - cal->x_min;
	int result = 0;

	if (raw > cal->x_zero) {
		int diff = raw - cal->x_zero;
		if (rangePos != 0) {
			result = (diff * MIYOO_AXIS_MAX) / rangePos;
			if (result > MIYOO_AXIS_MAX)
				result = MIYOO_AXIS_MAX;
		}
	} else if (raw < cal->x_zero) {
		int diff = cal->x_zero - raw;
		if (rangeNeg != 0) {
			result = -((diff * -MIYOO_AXIS_MIN) / rangeNeg);
			if (result < MIYOO_AXIS_MIN)
				result = MIYOO_AXIS_MIN;
		}
	}

	return result;
}

static int miyoo_calibrate_axis_y(const struct miyoo_cal *cal, int raw)
{
	int rangePos = cal->y_max - cal->y_zero;
	int rangeNeg = cal->y_zero - cal->y_min;
	int result = 0;

	if (raw > cal->y_zero) {
		int diff = raw - cal->y_zero;
		if (rangePos != 0) {
			result = (diff * MIYOO_AXIS_MAX) / rangePos;
			if (result > MIYOO_AXIS_MAX)
				result = MIYOO_AXIS_MAX;
		}
	} else if (raw < cal->y_zero) {
		int diff = cal->y_zero - raw;
		if (rangeNeg != 0) {
			result = -((diff * -MIYOO_AXIS_MIN) / rangeNeg);
			if (result < MIYOO_AXIS_MIN)
				result = MIYOO_AXIS_MIN;
		}
	}

	return result;
}

static void miyoo_apply_deadzone(int *px, int *py)
{
	long long x = *px, y = *py;
	long long magSq = x * x + y * y;
	long long dead = (long long)(0.10f * MIYOO_AXIS_MAX); /* 10% deadzone */
	long long deadSq = dead * dead;

	if (magSq <= deadSq) {
		*px = 0;
		*py = 0;
	}
}

/* Quickly auto-calibrate neutral from multiple frames. */
static int miyoo_autocal(struct joypad *joypad)
{
	long sumYL = 0, sumXL = 0, sumYR = 0, sumXR = 0;
	int framesRead = 0;
	unsigned char buf[MIYOO_FRAME_SIZE];

	while (framesRead < MIYOO_CALIBRATION_FRAMES) {
		ssize_t n = kernel_read(joypad->miyoo_filp, buf, MIYOO_FRAME_SIZE,
					&joypad->miyoo_filp->f_pos);
		if (n < 0) {
			dev_err(joypad->dev, "Auto-cal read error: %zd\n", n);
			msleep(10);
			continue;
		}
		if (n < MIYOO_FRAME_SIZE) {
			msleep(10);
			continue;
		}
		if (buf[0] == MIYOO_MAGIC_START && buf[5] == MIYOO_MAGIC_END) {
			sumYL += buf[1];
			sumXL += buf[2];
			sumYR += buf[3];
			sumXR += buf[4];
			framesRead++;
		}
	}

	if (!framesRead) {
		dev_err(joypad->dev, "No frames read for calibration!\n");
		return -EIO;
	}

	joypad->miyoo.left_cal.x_zero  = sumXL / framesRead;
	joypad->miyoo.left_cal.y_zero  = sumYL / framesRead;
	joypad->miyoo.right_cal.x_zero = sumXR / framesRead;
	joypad->miyoo.right_cal.y_zero = sumYR / framesRead;

	return 0;
}

/* Frame parser for the Miyoo data. */
static void miyoo_parse_frame(struct joypad *joypad, unsigned char *frame)
{
	if (frame[0] != MIYOO_MAGIC_START || frame[5] != MIYOO_MAGIC_END)
		return;

	joypad->last_data_jiffies = jiffies;

	{
		int rawYL = frame[1];
		int rawXL = frame[2];
		int rawYR = frame[3];
		int rawXR = frame[4];

		int lx = miyoo_calibrate_axis_x(&joypad->miyoo.left_cal, rawXL);
		int ly = miyoo_calibrate_axis_y(&joypad->miyoo.left_cal, rawYL);
		int rx = miyoo_calibrate_axis_x(&joypad->miyoo.right_cal, rawXR);
		int ry = miyoo_calibrate_axis_y(&joypad->miyoo.right_cal, rawYR);

		miyoo_apply_deadzone(&lx, &ly);
		miyoo_apply_deadzone(&rx, &ry);

		joypad->miyoo.left_x  = lx;
		joypad->miyoo.left_y  = ly;
		joypad->miyoo.right_x = rx;
		joypad->miyoo.right_y = ry;
	}
}

static void set_tty_9600_8N1(struct file *filp)
{
	struct tty_file_private *file_priv;
	struct tty_struct *tty;
	struct ktermios newt;

	if (!filp || !filp->private_data)
		return;

	file_priv = filp->private_data;
	if (!file_priv || !file_priv->tty)
		return;

	tty = file_priv->tty;
	newt = tty->termios;

	newt.c_iflag = 0;
	newt.c_oflag = 0;
	newt.c_cflag = B9600 | CS8 | CREAD | CLOCAL;
	newt.c_lflag = 0;

	memset(newt.c_cc, 0, NCCS);
	newt.c_cc[VMIN]  = 1;
	newt.c_cc[VTIME] = 0;

	tty_set_termios(tty, &newt);
}

static int miyoo_serial_open(struct joypad *joypad)
{
	struct file *filp;

	filp = filp_open("/dev/ttyS1", O_RDWR | O_NOCTTY, 0);
	if (IS_ERR(filp)) {
		dev_err(joypad->dev, "ERROR: open /dev/ttyS1 failed: %ld\n", PTR_ERR(filp));
		return PTR_ERR(filp);
	}

	joypad->miyoo_filp = filp;
	set_tty_9600_8N1(filp);
	return 0;
}

static void miyoo_serial_close(struct joypad *joypad)
{
	if (joypad->miyoo_filp) {
		filp_close(joypad->miyoo_filp, NULL);
		joypad->miyoo_filp = NULL;
	}
}

static int miyoo_serial_threadfn(void *data)
{
	struct joypad *joypad = data;

	while (!kthread_should_stop()) {
		unsigned char frame[MIYOO_FRAME_SIZE];
		ssize_t n;

		if (!joypad->miyoo_filp) {
			msleep(100);
			continue;
		}

		n = kernel_read(joypad->miyoo_filp, frame, MIYOO_FRAME_SIZE,
				&joypad->miyoo_filp->f_pos);
		if (n < 0) {
			dev_err(joypad->dev, "Serial read error: %zd\n", n);
			msleep(10);
			continue;
		}
		if (n < MIYOO_FRAME_SIZE) {
			msleep(10);
			continue;
		}

		miyoo_parse_frame(joypad, frame);

		/* Re-open TTY if no data for 5 seconds. */
		if (time_after(jiffies, joypad->last_data_jiffies + MIYOO_SERIAL_INACTIVITY_TIMEOUT)) {
			dev_warn(joypad->dev, "No data for 5s; re-opening TTY.\n");
			miyoo_serial_close(joypad);
			if (!miyoo_serial_open(joypad))
				joypad->last_data_jiffies = jiffies;
			else
				dev_err(joypad->dev, "Failed to re-open /dev/ttyS1.\n");
		}
	}

	return 0;
}

static int miyoo_start_serial(struct joypad *joypad)
{
	int ret;

	ret = miyoo_serial_open(joypad);
	if (ret < 0)
		return ret;

	ret = miyoo_autocal(joypad);
	if (ret < 0)
		dev_err(joypad->dev, "Auto-calibration failed: %d\n", ret);

	joypad->miyoo_thread = kthread_run(miyoo_serial_threadfn, joypad,
					   "miyoo-serial-thread");
	if (IS_ERR(joypad->miyoo_thread)) {
		miyoo_serial_close(joypad);
		return PTR_ERR(joypad->miyoo_thread);
	}

	joypad->last_data_jiffies = jiffies;
	return 0;
}

static void miyoo_delayed_init_workfn(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct joypad *joypad = container_of(dwork, struct joypad, miyoo_init_work);

	miyoo_start_serial(joypad);
}

/* -------------------------------------------------------------------------- */
/*                          Input Device Callbacks                            */
/* -------------------------------------------------------------------------- */

static void joypad_poll(struct input_polled_dev *poll_dev)
{
	struct joypad *joypad = poll_dev->private;

	if (!joypad->enable)
		goto update_interval;

	switch (joypad->mode) {
	case JOYPAD_MODE_MULTIADC:
		joypad_adc_check_multiadc(poll_dev);
		joypad_gpio_check(joypad);
		break;

	case JOYPAD_MODE_SINGLEADC:
		joypad_adc_check_singleadc(poll_dev);
		joypad_gpio_check(joypad);
		break;

	case JOYPAD_MODE_MIYOOSERIAL:
		/* Report the last known values from the Miyoo thread. */
		input_report_abs(poll_dev->input, ABS_X,  joypad->miyoo.left_x);
		input_report_abs(poll_dev->input, ABS_Y,  joypad->miyoo.left_y);
		input_report_abs(poll_dev->input, ABS_RX, joypad->miyoo.right_x);
		input_report_abs(poll_dev->input, ABS_RY, joypad->miyoo.right_y);
		input_sync(poll_dev->input);

		joypad_gpio_check(joypad);
		break;

	case JOYPAD_MODE_NONE:
	default:
		joypad_gpio_check(joypad);
		break;
	}

update_interval:
	if (poll_dev->poll_interval != joypad->poll_interval) {
		mutex_lock(&joypad->lock);
		poll_dev->poll_interval = joypad->poll_interval;
		mutex_unlock(&joypad->lock);
	}
}

static void joypad_open(struct input_polled_dev *poll_dev)
{
	struct joypad *joypad = poll_dev->private;
	int i;

	/* Reset old_value for GPIOs. */
	for (i = 0; i < joypad->bt_gpio_count; i++) {
		struct bt_gpio *gpio = &joypad->gpios[i];
		gpio->old_value = (gpio->active_level ? 0 : 1);
	}

	/* Perform initial ADC calibration reading. */
	switch (joypad->mode) {
	case JOYPAD_MODE_MULTIADC:
		for (i = 0; i < joypad->chan_count; i++) {
			int val;
			struct bt_adc *adc = &joypad->adcs[i];
			val = joypad_adc_read_multi(joypad, adc);
			adc->cal = val; /* store initial offset */
		}
		joypad_adc_check_multiadc(poll_dev);
		joypad_gpio_check(joypad);
		break;

	case JOYPAD_MODE_SINGLEADC:
		for (i = 0; i < joypad->amux_count; i++) {
			int val;
			struct bt_adc_single *adc = &joypad->adcs_single[i];
			val = joypad_adc_read_single(joypad->amux, adc);
			adc->cal = val;
		}
		joypad_adc_check_singleadc(poll_dev);
		joypad_gpio_check(joypad);
		break;

	case JOYPAD_MODE_MIYOOSERIAL:
		joypad_gpio_check(joypad);
		break;

	case JOYPAD_MODE_NONE:
	default:
		joypad_gpio_check(joypad);
		break;
	}

	mutex_lock(&joypad->lock);
	joypad->enable = true;
	mutex_unlock(&joypad->lock);
}

static void joypad_close(struct input_polled_dev *poll_dev)
{
	struct joypad *joypad = poll_dev->private;

	if (joypad->has_rumble) {
		cancel_work_sync(&joypad->play_work);
		pwm_vibrator_stop(joypad);
	}

	mutex_lock(&joypad->lock);
	joypad->enable = false;
	mutex_unlock(&joypad->lock);
}

/* -------------------------------------------------------------------------- */
/*                      Device Tree / Platform Setup                          */
/* -------------------------------------------------------------------------- */

static int joypad_rumble_setup(struct device *dev, struct joypad *joypad)
{
	int error;
	struct pwm_state state;

	joypad->pwm = devm_pwm_get(dev, "enable");
	if (IS_ERR(joypad->pwm)) {
		dev_err(dev, "No valid PWM found for rumble\n");
		return PTR_ERR(joypad->pwm);
	}

	INIT_WORK(&joypad->play_work, pwm_vibrator_play_work);

	pwm_init_state(joypad->pwm, &state);
	state.enabled = false;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0))
	error = pwm_apply_state(joypad->pwm, &state);
#else
	error = pwm_apply_might_sleep(joypad->pwm, &state);
#endif
	if (error) {
		dev_err(dev, "failed to apply initial PWM state: %d\n", error);
		return error;
	}

	return 0;
}

static int joypad_iochannel_setup(struct device *dev, struct joypad *joypad)
{
	enum iio_chan_type type;
	const char *uname;
	int i, ret;

	for (i = 0; i < joypad->chan_count; i++) {
		ret = of_property_read_string_index(dev->of_node, "io-channel-names", i, &uname);
		if (ret < 0) {
			dev_err(dev, "invalid channel name index[%d]\n", i);
			return -EINVAL;
		}

		joypad->adc_ch[i] = devm_iio_channel_get(dev, uname);
		if (IS_ERR(joypad->adc_ch[i])) {
			dev_err(dev, "iio channel get error\n");
			return PTR_ERR(joypad->adc_ch[i]);
		}

		if (!joypad->adc_ch[i]->indio_dev)
			return -ENXIO;

		if (iio_get_channel_type(joypad->adc_ch[i], &type))
			return -EINVAL;

		if (type != IIO_VOLTAGE) {
			dev_err(dev, "Incompatible channel type %d\n", type);
			return -EINVAL;
		}
	}
	return 0;
}

static int joypad_adc_setup_multi(struct device *dev, struct joypad *joypad)
{
	int i;

	joypad->adcs = devm_kzalloc(dev,
				    joypad->chan_count * sizeof(struct bt_adc),
				    GFP_KERNEL);
	if (!joypad->adcs)
		return -ENOMEM;

	for (i = 0; i < joypad->chan_count; i++) {
		struct bt_adc *adc = &joypad->adcs[i];

		adc->scale = joypad->bt_adc_scale;
		adc->max   = (ADC_MAX_VOLTAGE / 2);
		adc->min   = -adc->max;

		if (adc->scale) {
			adc->max *= adc->scale;
			adc->min *= adc->scale;
		}

		adc->channel = i;
		adc->invert = false; /* will override below if set */

		/* Assign default tuning, override via DT if present. */
		adc->tuning_p = ADC_TUNING_DEFAULT;
		adc->tuning_n = ADC_TUNING_DEFAULT;

		/*
		 * Example: channels in order
		 * 0 -> ABS_RY, 1 -> ABS_RX, 2 -> ABS_Y, 3 -> ABS_X,
		 * 4 -> ABS_Z,  5 -> ABS_RZ
		 * The actual usage depends on your hardware.
		 */
		switch (i) {
		case 0:
			adc->invert = joypad->invert_absry;
			adc->report_type = ABS_RY;
			device_property_read_u32(dev, "abs_ry-p-tuning", &adc->tuning_p);
			device_property_read_u32(dev, "abs_ry-n-tuning", &adc->tuning_n);
			break;
		case 1:
			adc->invert = joypad->invert_absrx;
			adc->report_type = ABS_RX;
			device_property_read_u32(dev, "abs_rx-p-tuning", &adc->tuning_p);
			device_property_read_u32(dev, "abs_rx-n-tuning", &adc->tuning_n);
			break;
		case 2:
			adc->invert = joypad->invert_absy;
			adc->report_type = ABS_Y;
			device_property_read_u32(dev, "abs_y-p-tuning", &adc->tuning_p);
			device_property_read_u32(dev, "abs_y-n-tuning", &adc->tuning_n);
			break;
		case 3:
			adc->invert = joypad->invert_absx;
			adc->report_type = ABS_X;
			device_property_read_u32(dev, "abs_x-p-tuning", &adc->tuning_p);
			device_property_read_u32(dev, "abs_x-n-tuning", &adc->tuning_n);
			break;
		case 4:
			adc->invert = joypad->invert_absz;
			adc->report_type = ABS_Z;
			device_property_read_u32(dev, "abs_z-p-tuning", &adc->tuning_p);
			device_property_read_u32(dev, "abs_z-n-tuning", &adc->tuning_n);
			break;
		case 5:
			adc->invert = joypad->invert_absrz;
			adc->report_type = ABS_RZ;
			device_property_read_u32(dev, "abs_rz-p-tuning", &adc->tuning_p);
			device_property_read_u32(dev, "abs_rz-n-tuning", &adc->tuning_n);
			break;
		default:
			dev_err(dev, "Too many channels or unrecognized index %d\n", i);
			return -EINVAL;
		}
	}

	return 0;
}

static int joypad_amux_setup(struct device *dev, struct joypad *joypad)
{
	struct analog_mux *amux;
	enum iio_chan_type type;
	enum of_gpio_flags flags;
	int ret;

	amux = devm_kzalloc(dev, sizeof(*amux), GFP_KERNEL);
	if (!amux)
		return -ENOMEM;

	amux->iio_ch = devm_iio_channel_get(dev, "amux_adc");
	if (IS_ERR(amux->iio_ch)) {
		ret = PTR_ERR(amux->iio_ch);
		dev_err(dev, "Failed to get single-ADC channel, err=%d\n", ret);
		return ret;
	}

	if (!amux->iio_ch->indio_dev)
		return -ENXIO;

	if (iio_get_channel_type(amux->iio_ch, &type))
		return -EINVAL;

	if (type != IIO_VOLTAGE) {
		dev_err(dev, "Incompatible channel type %d\n", type);
		return -EINVAL;
	}

	amux->sel_a_gpio = of_get_named_gpio_flags(dev->of_node, "amux-a-gpios", 0, &flags);
	if (gpio_is_valid(amux->sel_a_gpio)) {
		ret = devm_gpio_request(dev, amux->sel_a_gpio, "amux-sel-a");
		if (ret < 0)
			return ret;
		ret = gpio_direction_output(amux->sel_a_gpio, 0);
		if (ret < 0)
			return ret;
	}

	amux->sel_b_gpio = of_get_named_gpio_flags(dev->of_node, "amux-b-gpios", 0, &flags);
	if (gpio_is_valid(amux->sel_b_gpio)) {
		ret = devm_gpio_request(dev, amux->sel_b_gpio, "amux-sel-b");
		if (ret < 0)
			return ret;
		ret = gpio_direction_output(amux->sel_b_gpio, 0);
		if (ret < 0)
			return ret;
	}

	amux->en_gpio = of_get_named_gpio_flags(dev->of_node, "amux-en-gpios", 0, &flags);
	if (gpio_is_valid(amux->en_gpio)) {
		ret = devm_gpio_request(dev, amux->en_gpio, "amux-en");
		if (ret < 0)
			return ret;
		ret = gpio_direction_output(amux->en_gpio, 0);
		if (ret < 0)
			return ret;
	}

	joypad->amux = amux;
	return 0;
}

static int joypad_adc_setup_single(struct device *dev, struct joypad *joypad)
{
	int i;
	u32 channel_mapping[4] = {0, 1, 2, 3};

	if (device_property_present(dev, "amux-channel-mapping")) {
		int ret = of_property_read_u32_array(dev->of_node,
				"amux-channel-mapping", channel_mapping, 4);
		if (ret < 0) {
			dev_err(dev, "Invalid channel mapping\n");
			return ret;
		}
	}

	joypad->adcs_single = devm_kzalloc(dev,
				joypad->amux_count * sizeof(struct bt_adc_single),
				GFP_KERNEL);
	if (!joypad->adcs_single)
		return -ENOMEM;

	for (i = 0; i < joypad->amux_count; i++) {
		struct bt_adc_single *adc = &joypad->adcs_single[i];

		adc->scale = joypad->bt_adc_scale;
		adc->max   = ADC_MAX_VOLTAGE / 2;
		adc->min   = -adc->max;

		if (adc->scale) {
			adc->max *= adc->scale;
			adc->min *= adc->scale;
		}

		adc->amux_ch = channel_mapping[i];
		adc->invert  = false;

		adc->tuning_p = ADC_TUNING_DEFAULT;
		adc->tuning_n = ADC_TUNING_DEFAULT;

		switch (i) {
		case 0:
			adc->invert = joypad->invert_absry;
			adc->report_type = ABS_RY;
			device_property_read_u32(dev, "abs_ry-p-tuning", &adc->tuning_p);
			device_property_read_u32(dev, "abs_ry-n-tuning", &adc->tuning_n);
			break;
		case 1:
			adc->invert = joypad->invert_absrx;
			adc->report_type = ABS_RX;
			device_property_read_u32(dev, "abs_rx-p-tuning", &adc->tuning_p);
			device_property_read_u32(dev, "abs_rx-n-tuning", &adc->tuning_n);
			break;
		case 2:
			adc->invert = joypad->invert_absy;
			adc->report_type = ABS_Y;
			device_property_read_u32(dev, "abs_y-p-tuning", &adc->tuning_p);
			device_property_read_u32(dev, "abs_y-n-tuning", &adc->tuning_n);
			break;
		case 3:
			adc->invert = joypad->invert_absx;
			adc->report_type = ABS_X;
			device_property_read_u32(dev, "abs_x-p-tuning", &adc->tuning_p);
			device_property_read_u32(dev, "abs_x-n-tuning", &adc->tuning_n);
			break;
		default:
			dev_err(dev, "amux-count mismatch (index %d)\n", i);
			return -EINVAL;
		}
	}

	return 0;
}

static int joypad_gpio_setup(struct device *dev, struct joypad *joypad)
{
	struct device_node *node = dev->of_node;
	struct device_node *pp;
	int i = 0;

	if (!node)
		return -ENODEV;

	joypad->gpios = devm_kzalloc(dev, joypad->bt_gpio_count * sizeof(struct bt_gpio),
				     GFP_KERNEL);
	if (!joypad->gpios)
		return -ENOMEM;

	for_each_child_of_node(node, pp) {
		enum of_gpio_flags flags;
		struct bt_gpio *gpio = &joypad->gpios[i];
		int error;

		gpio->num = of_get_gpio_flags(pp, 0, &flags);
		if (gpio->num < 0) {
			error = gpio->num;
			dev_err(dev, "Failed to get gpio flags, error: %d\n", error);
			return error;
		}

		gpio->active_level = !(flags & OF_GPIO_ACTIVE_LOW);
		gpio->label = of_get_property(pp, "label", NULL);

		if (gpio_is_valid(gpio->num)) {
			error = devm_gpio_request_one(dev, gpio->num, GPIOF_IN, gpio->label);
			if (error < 0) {
				dev_err(dev, "Failed to request GPIO %d, error %d\n",
					gpio->num, error);
				return error;
			}
		}

		if (of_property_read_u32(pp, "linux,code", &gpio->linux_code)) {
			dev_err(dev, "Button missing linux,code property\n");
			return -EINVAL;
		}

		if (of_property_read_u32(pp, "linux,input-type", &gpio->report_type))
			gpio->report_type = EV_KEY;

		i++;
	}

	if (!i)
		return -EINVAL;

	return 0;
}

static int joypad_dt_parse_unified(struct device *dev, struct joypad *joypad)
{
	const char *mode_str = "none";

	joypad->poll_interval = 10;
	joypad->bt_adc_scale  = 1;

	/* Key property that dictates the driver mode. */
	device_property_read_string(dev, "joypad-mode", &mode_str);
	if (!strcmp(mode_str, "multiadc"))
		joypad->mode = JOYPAD_MODE_MULTIADC;
	else if (!strcmp(mode_str, "singleadc"))
		joypad->mode = JOYPAD_MODE_SINGLEADC;
	else if (!strcmp(mode_str, "miyooserial"))
		joypad->mode = JOYPAD_MODE_MIYOOSERIAL;
	else
		joypad->mode = JOYPAD_MODE_NONE;

	device_property_read_u32(dev, "poll-interval", &joypad->poll_interval);
	device_property_read_u32(dev, "button-adc-fuzz", &joypad->bt_adc_fuzz);
	device_property_read_u32(dev, "button-adc-flat", &joypad->bt_adc_flat);
	device_property_read_u32(dev, "button-adc-scale", &joypad->bt_adc_scale);
	device_property_read_u32(dev, "button-adc-deadzone", &joypad->bt_adc_deadzone);

	joypad->auto_repeat   = device_property_present(dev, "autorepeat");
	joypad->invert_absx   = device_property_present(dev, "invert-absx");
	joypad->invert_absy   = device_property_present(dev, "invert-absy");
	joypad->invert_absz   = device_property_present(dev, "invert-absz");
	joypad->invert_absrx  = device_property_present(dev, "invert-absrx");
	joypad->invert_absry  = device_property_present(dev, "invert-absry");
	joypad->invert_absrz  = device_property_present(dev, "invert-absrz");

	joypad->bt_gpio_count = device_get_child_node_count(dev);

	joypad->has_rumble = device_property_present(dev, "pwm-names");

	switch (joypad->mode) {
	case JOYPAD_MODE_MULTIADC:
		joypad->chan_count = of_property_count_strings(dev->of_node, "io-channel-names");
		if (joypad->chan_count < 0) {
			dev_err(dev, "No ADC channels for multiadc\n");
			return -EINVAL;
		}
		break;

	case JOYPAD_MODE_SINGLEADC:
		if (device_property_read_u32(dev, "amux-count", &joypad->amux_count)) {
			dev_err(dev, "No valid amux-count for singleadc\n");
			return -EINVAL;
		}
		break;

	case JOYPAD_MODE_MIYOOSERIAL:
		/* Setup default calibrations (override if you want). */
		joypad->miyoo.left_cal.x_min  = 85;
		joypad->miyoo.left_cal.x_max  = 200;
		joypad->miyoo.left_cal.x_zero = 130;
		joypad->miyoo.left_cal.y_min  = 85;
		joypad->miyoo.left_cal.y_max  = 200;
		joypad->miyoo.left_cal.y_zero = 130;

		joypad->miyoo.right_cal.x_min  = 85;
		joypad->miyoo.right_cal.x_max  = 200;
		joypad->miyoo.right_cal.x_zero = 130;
		joypad->miyoo.right_cal.y_min  = 85;
		joypad->miyoo.right_cal.y_max  = 200;
		joypad->miyoo.right_cal.y_zero = 130;
		break;

	case JOYPAD_MODE_NONE:
	default:
		break;
	}

	if (joypad_gpio_setup(dev, joypad))
		return -EINVAL;

	return 0;
}

static int joypad_input_setup(struct device *dev, struct joypad *joypad)
{
	struct input_polled_dev *poll_dev;
	struct input_dev *input;
	int error, i;
	u32 vendor = 0, revision = 0, product = 0;

	poll_dev = devm_input_allocate_polled_device(dev);
	if (!poll_dev) {
		dev_err(dev, "no memory for polled device\n");
		return -ENOMEM;
	}

	poll_dev->private = joypad;
	poll_dev->poll_interval = joypad->poll_interval;
	poll_dev->poll  = joypad_poll;
	poll_dev->open  = joypad_open;
	poll_dev->close = joypad_close;
	joypad->poll_dev = poll_dev;

	input = poll_dev->input;
	joypad->input = input;

	input->name = DRV_NAME;
	joypad_input_g = input; /* global reference if needed */

	device_property_read_string(dev, "joypad-name", &input->name);
	input->phys = DRV_NAME "/input0";

	device_property_read_u32(dev, "joypad-vendor", &vendor);
	device_property_read_u32(dev, "joypad-revision", &revision);
	device_property_read_u32(dev, "joypad-product", &product);

	input->id.bustype = BUS_HOST;
	input->id.vendor  = (u16)vendor;
	input->id.product = (u16)product;
	input->id.version = (u16)revision;

	/* Optional auto-repeat support for keys. */
	if (joypad->auto_repeat)
		__set_bit(EV_REP, input->evbit);

	__set_bit(EV_KEY, input->evbit);

	/* Set up the GPIO-based keys. */
	for (i = 0; i < joypad->bt_gpio_count; i++) {
		struct bt_gpio *gpio = &joypad->gpios[i];
		input_set_capability(input, gpio->report_type, gpio->linux_code);
	}

	/* Based on mode, set up the relevant ABS axes. */
	if (joypad->mode == JOYPAD_MODE_MULTIADC) {
		__set_bit(EV_ABS, input->evbit);

		for (i = 0; i < joypad->chan_count; i++) {
			struct bt_adc *adc = &joypad->adcs[i];
			input_set_abs_params(input, adc->report_type,
				adc->min, adc->max,
				joypad->bt_adc_fuzz, joypad->bt_adc_flat);
		}
	} else if (joypad->mode == JOYPAD_MODE_SINGLEADC) {
		__set_bit(EV_ABS, input->evbit);

		for (i = 0; i < joypad->amux_count; i++) {
			struct bt_adc_single *adc = &joypad->adcs_single[i];
			input_set_abs_params(input, adc->report_type,
				adc->min, adc->max,
				joypad->bt_adc_fuzz, joypad->bt_adc_flat);
		}
	} else if (joypad->mode == JOYPAD_MODE_MIYOOSERIAL) {
		__set_bit(EV_ABS, input->evbit);

		input_set_abs_params(input, ABS_X,  MIYOO_AXIS_MIN, MIYOO_AXIS_MAX, 16, 16);
		input_set_abs_params(input, ABS_Y,  MIYOO_AXIS_MIN, MIYOO_AXIS_MAX, 16, 16);
		input_set_abs_params(input, ABS_RX, MIYOO_AXIS_MIN, MIYOO_AXIS_MAX, 16, 16);
		input_set_abs_params(input, ABS_RY, MIYOO_AXIS_MIN, MIYOO_AXIS_MAX, 16, 16);
	}

	/* Optional rumble support. */
	if (joypad->has_rumble) {
		input_set_capability(input, EV_FF, FF_RUMBLE);
		error = input_ff_create_memless(input, joypad, rumble_play_effect);
		if (error) {
			dev_err(dev, "unable to register rumble, err=%d\n", error);
			return error;
		}
	}

	error = input_register_polled_device(poll_dev);
	if (error) {
		dev_err(dev, "unable to register polled device, err=%d\n", error);
		return error;
	}

	return 0;
}

/* -------------------------------------------------------------------------- */
/*                               PM Operations                                */
/* -------------------------------------------------------------------------- */

static int __maybe_unused joypad_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);

	if (joypad->has_rumble) {
		cancel_work_sync(&joypad->play_work);
		pwm_vibrator_stop(joypad);
	}

	return 0;
}

static int __maybe_unused joypad_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);

	if (joypad->has_rumble && joypad->level)
		pwm_vibrator_start(joypad);

	return 0;
}

static SIMPLE_DEV_PM_OPS(joypad_pm_ops, joypad_suspend, joypad_resume);

/* -------------------------------------------------------------------------- */
/*                             Probe / Remove                                 */
/* -------------------------------------------------------------------------- */

static int joypad_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct joypad *joypad;
	int error;

	joypad = devm_kzalloc(dev, sizeof(*joypad), GFP_KERNEL);
	if (!joypad)
		return -ENOMEM;

	mutex_init(&joypad->lock);
	platform_set_drvdata(pdev, joypad);
	joypad->dev = dev;

	/* Parse device tree */
	error = joypad_dt_parse_unified(dev, joypad);
	if (error) {
		dev_err(dev, "DT parse error!(err = %d)\n", error);
		return error;
	}

	/* If multi-ADC, get channels and prepare data structures. */
	if (joypad->mode == JOYPAD_MODE_MULTIADC) {
		error = joypad_adc_setup_multi(dev, joypad);
		if (error)
			return error;

		error = joypad_iochannel_setup(dev, joypad);
		if (error)
			return error;
	}
	/* If single-ADC, set up multiplexer and channel data. */
	else if (joypad->mode == JOYPAD_MODE_SINGLEADC) {
		error = joypad_amux_setup(dev, joypad);
		if (error)
			return error;

		error = joypad_adc_setup_single(dev, joypad);
		if (error)
			return error;
	}

	/* Set up input device. */
	error = joypad_input_setup(dev, joypad);
	if (error) {
		dev_err(dev, "Input setup failed!(err = %d)\n", error);
		return error;
	}

	/* If rumble is present, create sysfs group and set up PWM. */
	if (joypad->has_rumble) {
		joypad->rumble_enabled = true;
		error = sysfs_create_group(&pdev->dev.kobj, &joypad_rumble_attr_group);
		if (error) {
			dev_err(dev, "create sysfs group fail, error: %d\n", error);
			return error;
		}
		error = joypad_rumble_setup(dev, joypad);
		if (error) {
			dev_err(dev, "Rumble setup failed!(err = %d)\n", error);
			return error;
		}
	}

	/* If Miyoo serial mode, do a delayed init. */
	if (joypad->mode == JOYPAD_MODE_MIYOOSERIAL) {
		INIT_DELAYED_WORK(&joypad->miyoo_init_work, miyoo_delayed_init_workfn);
		/* 10 seconds delayed start to allow TTY system to come up. */
		schedule_delayed_work(&joypad->miyoo_init_work, 10 * HZ);
	}

	dev_info(dev, "probe success, mode=%d\n", joypad->mode);
	return 0;
}

static void joypad_remove(struct platform_device *pdev)
{
	struct joypad *joypad = platform_get_drvdata(pdev);

	if (!joypad)
		return;

	if (joypad->has_rumble)
		sysfs_remove_group(&pdev->dev.kobj, &joypad_rumble_attr_group);

	/* Stop Miyoo thread if used. */
	if (joypad->mode == JOYPAD_MODE_MIYOOSERIAL) {
		cancel_delayed_work_sync(&joypad->miyoo_init_work);
		if (joypad->miyoo_thread) {
			kthread_stop(joypad->miyoo_thread);
			joypad->miyoo_thread = NULL;
		}
		miyoo_serial_close(joypad);
	}
}

/* -------------------------------------------------------------------------- */
/*                               Match Table                                  */
/* -------------------------------------------------------------------------- */

static const struct of_device_id joypad_of_match[] = {
	{ .compatible = "open,joypad", },
	{ }
};
MODULE_DEVICE_TABLE(of, joypad_of_match);

static struct platform_driver joypad_driver = {
	.probe          = joypad_probe,
	.remove_new     = joypad_remove,
	.driver         = {
		.name           = DRV_NAME,
		.pm             = &joypad_pm_ops,
		.of_match_table = of_match_ptr(joypad_of_match),
	},
};

static int __init joypad_init(void)
{
	return platform_driver_register(&joypad_driver);
}

static void __exit joypad_exit(void)
{
	platform_driver_unregister(&joypad_driver);
}

late_initcall(joypad_init);
module_exit(joypad_exit);

MODULE_AUTHOR("spycat");
MODULE_DESCRIPTION("Joypad driver for multiple handheld gaming devices");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_INFO(intree, "Y");
