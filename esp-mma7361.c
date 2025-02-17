#include <stdio.h>

#include "MMA7361.h"

#include "esp_check.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"


#define G15 800
#define G60 206

static const char *TAG = "MMA7361";

typedef struct mma7361_t mma7361_t;

struct mma7361_t{
    adc_channel_t x_channel;
    adc_channel_t y_channel;
    adc_channel_t z_channel;
    int x_offset;
    int y_offset;
    int z_offset;
    unsigned int cali; 
    gpio_num_t zero_g;
    gpio_num_t g_select;
    gpio_num_t self_test;
    gpio_num_t sleep;
    adc_oneshot_unit_handle_t adc1_handle;
    adc_cali_handle_t adc1_cali_handle;
};

static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAGADC, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Calibration Success");
    }
    else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated)
    {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    }
    else
    {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void adc_calibration_deinit(adc_cali_handle_t handle)

{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

esp_err_t mma7361_new(mma7361_config_t *config, mma7361_handle_t *ret_ma7361){

    esp_err_t ret = ESP_OK;
    mma7361_t *mma7361 = NULL;
    ESP_GOTO_ON_FALSE(config && ret_ma7361, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    mma7361 = calloc(1,sizeof(mma7361_t));
    
    mma7361->x_channel  = config->x_channel;
    mma7361->y_channel  = config->y_channel;
    mma7361->z_channel  = config->z_channel;
    mma7361->zero_g     = config->zero_g;
    mma7361->g_select   = config->g_select;
    mma7361->self_test  = config->self_test;
    mma7361->sleep      = config->sleep;
    mma7361->x_offset  = config->x_offset;
    mma7361->y_offset  = config->y_offset;
    mma7361->z_offset  = config->z_offset;
    
    uint64_t gpio_pin_sel = (1ULL<<mma7361->g_select) |  (1ULL<<config->self_test) | (1ULL<<config->sleep);

    printf("%lld\n", gpio_pin_sel);

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = gpio_pin_sel;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_pin_sel = (1ULL<<config->zero_g);
    mma7361->cali = 800;
    io_conf.pin_bit_mask = gpio_pin_sel;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_set_level(mma7361->g_select,0);
    mma7361->cali = G15;
    gpio_set_level(mma7361->self_test,0);
    gpio_set_level(mma7361->sleep,1);  

    mma7361->adc1_handle = NULL;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &mma7361->adc1_handle));

    adc_oneshot_chan_cfg_t config_adc = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(mma7361->adc1_handle, mma7361->x_channel, &config_adc));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(mma7361->adc1_handle, mma7361->y_channel, &config_adc));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(mma7361->adc1_handle, mma7361->z_channel, &config_adc));

    mma7361->adc1_cali_handle = NULL;
    adc_calibration_init(ADC_UNIT_1, config_adc.atten, &mma7361->adc1_cali_handle );

    *ret_ma7361 = mma7361;
    return ret;

err:
    if(mma7361){
        free(mma7361);
    }
    return ret;
}

void mma7361_read_3axes(int *axis_x, int *axis_y, int *axis_z, mma7361_handle_t mma7361_handle){
    
    int raw_axis_x, raw_axis_y, raw_axis_z;
    
    ESP_ERROR_CHECK(adc_oneshot_read(mma7361_handle->adc1_handle, mma7361_handle->x_channel, &raw_axis_x));
    ESP_ERROR_CHECK(adc_oneshot_read(mma7361_handle->adc1_handle, mma7361_handle->y_channel, &raw_axis_y));
    ESP_ERROR_CHECK(adc_oneshot_read(mma7361_handle->adc1_handle, mma7361_handle->z_channel, &raw_axis_z));
                
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(mma7361_handle->adc1_cali_handle, raw_axis_x, axis_x));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(mma7361_handle->adc1_cali_handle, raw_axis_y, axis_y));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(mma7361_handle->adc1_cali_handle, raw_axis_z, axis_z));
    
    *axis_x = (int)(((float)(*axis_x)-mma7361_handle->x_offset)/((float)mma7361_handle->cali/1000));
    *axis_y = (int)(((float)(*axis_y)-mma7361_handle->y_offset)/((float)mma7361_handle->cali/1000));
    *axis_z = (int)(((float)(*axis_z)-mma7361_handle->z_offset)/((float)mma7361_handle->cali/1000));
    ESP_LOGI(TAG, "x: %.d, Y: %d, z: %d",*axis_x,*axis_y,*axis_z);
    
}

esp_err_t mma7361_del(mma7361_handle_t mma7361_handle)
{
    ESP_RETURN_ON_FALSE(mma7361_handle, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    
    adc_calibration_deinit(mma7361_handle->adc1_cali_handle);
    free(mma7361_handle->adc1_handle);
    free(mma7361_handle);
    ESP_LOGI(TAG, "mma7361 has been deleted!");
    return ESP_OK; 
}

void mma7361_gselect(uint32_t value, mma7361_handle_t mma7361_handle){
    gpio_set_level(mma7361_handle->g_select,value);
    mma7361_handle->cali = (!value) ? G15 : G60;
    ESP_LOGI(TAG, "G SELECT GAIN: %d", mma7361_handle->cali);

}

void mma7361_selftest(uint32_t value, mma7361_handle_t mma7361_handle){
    gpio_set_level(mma7361_handle->self_test,value);
    ESP_LOGI(TAG, "SELF TEST: %s", value ? "ON" : "OFF");
}

void mma7361_sleep(uint32_t value, mma7361_handle_t mma7361_handle){
    gpio_set_level(mma7361_handle->sleep,value);
    ESP_LOGI(TAG, "SLEEP: %s", !value ? "ON" : "OFF");  
}

