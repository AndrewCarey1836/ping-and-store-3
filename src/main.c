/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/* 
* Modified by Andrew Carey
*/

//general c
#include <stdio.h>

//multicell location 
#include <zephyr/kernel.h>
#include <modem/lte_lc.h>
#include <dk_buttons_and_leds.h>
#include <net/multicell_location.h>
//#include <date_time.h>

#include <zephyr/logging/log.h>

//included with storage
//#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/storage/disk_access.h>
//#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <ff.h>

//blink
#include <zephyr/drivers/gpio.h>

//button
//#include <dk_buttons_and_leds.h>
#include <modem/at_cmd_parser.h>
#include <modem/at_params.h>

//power
#include <zephyr/init.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <soc.h>
#include "retained.h"
#include <hal/nrf_gpio.h>

#define BUSY_WAIT_S 2U
#define SLEEP_S 2U

#define DEVICE_ID_MAX_LEN 64

//LOG_MODULE_REGISTER(multicell_location_sample, CONFIG_MULTICELL_LOCATION_SAMPLE_LOG_LEVEL);
//state what the log is called
//this was part of storage
LOG_MODULE_REGISTER(main);

//set up the file system
static FATFS fat_fs;
/* mounting info */
static struct fs_mount_t mp = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
};

static const char *disk_mount_pt = "/SD:";
static const char *testTxt = "/SD:/test.txt";
static const char *textTxt = "/SD:/text.txt";
static const char *derpTxt = "/SD:/derp.txt";

static const char *separator = "*******************************************\n";

/***
 * set up for communication with pico
*/
#define SLEEP_TIME_MS   100
#define TOGGLE_TIME 50
#define HIGH 1
#define LOW 0

/**
 * Output 
 */
#define Tower_Read_Success_Pin DT_ALIAS(led0)
#define Tower_Read_Fail_Pin DT_ALIAS(led1)
#define Tower_Buffer_Full_Pin DT_ALIAS(led2)
#define Tower_Connected_Pin DT_ALIAS(led3)
static const struct gpio_dt_spec Tower_Read_Success = GPIO_DT_SPEC_GET(Tower_Read_Success_Pin, gpios);
static const struct gpio_dt_spec Tower_Read_Fail = GPIO_DT_SPEC_GET(Tower_Read_Fail_Pin, gpios);
static const struct gpio_dt_spec Tower_Buffer_Full = GPIO_DT_SPEC_GET(Tower_Buffer_Full_Pin, gpios);
static const struct gpio_dt_spec Tower_Connected = GPIO_DT_SPEC_GET(Tower_Connected_Pin, gpios);

/****
 * Input
*/
/*
#define Power_Button_Pin DT_ALIAS(sw0)
#define Collect_Towers_Pin DT_ALIAS(sw1)
#define Store_Towers_Pin DT_ALIAS(sw2)
#define Output_Other_Pin DT_ALIAS(sw3)
*/
//#define Power_Button BIT(DK_BTN1)
//#define Power_Button BIT(DK_BTN4)
#define Collect_Towers BIT(DK_BTN1)
#define Store_Towers BIT(DK_BTN2)
#define Output_Other BIT(DK_BTN3)
/*
static const struct gpio_dt_spec Power_Button = GPIO_DT_SPEC_GET(Power_Button_Pin, gpios);
static const struct gpio_dt_spec Collect_Towers = GPIO_DT_SPEC_GET(Collect_Towers_Pin, gpios);
static const struct gpio_dt_spec Store_Towers = GPIO_DT_SPEC_GET(Store_Towers_Pin, gpios);
static const struct gpio_dt_spec Output_Other = GPIO_DT_SPEC_GET(Output_Other_Pin, gpios);
*/

//towers as a global string
//#define towerSize 200
#define towerSizeID 15
#define towerSizeMCCMNC 12
#define towerSizeTAC 12
#define towerSizeTA 12
#define towerSizeCOPS 25
#define towerSizeSNR 300

//towers

//tower 1
char t1ID[towerSizeID];
char t1MCC[towerSizeMCCMNC];
char t1MNC[towerSizeMCCMNC];
char t1TAC[towerSizeTAC];
char t1TA[towerSizeTA];
char t1COPS[towerSizeCOPS];
char t1RSRP[towerSizeMCCMNC];
char t1RSRQ[towerSizeMCCMNC];
char t1SNR[towerSizeSNR];

//tower 2
char t2ID[towerSizeID];
char t2MCC[towerSizeMCCMNC];
char t2MNC[towerSizeMCCMNC];
char t2TAC[towerSizeTAC];
char t2TA[towerSizeTA];
char t2COPS[towerSizeCOPS];
char t2RSRP[towerSizeMCCMNC];
char t2RSRQ[towerSizeMCCMNC];
char t2SNR[towerSizeSNR];

//tower 3
char t3ID[towerSizeID];
char t3MCC[towerSizeMCCMNC];
char t3MNC[towerSizeMCCMNC];
char t3TAC[towerSizeTAC];
char t3TA[towerSizeTA];
char t3COPS[towerSizeCOPS];
char t3RSRP[towerSizeMCCMNC];
char t3RSRQ[towerSizeMCCMNC];
char t3SNR[towerSizeSNR];

#define timeSize 50
char time[timeSize];

//list of known COPS
//a very lazy way of implementing,
//but i need it to work quit

//AT&T Mobility
static const char *att_mobility_cops="310410";

//T-Mobile
static const char *t_mobile_cops = "310260";

//AT&T FirstNet
static const char *firstnet_cops = "313100";

//Verizon
//static const char *verizon = "311480";

//other
static const char *verizon = "311490";

/*******
 * I am hoping this semaphore works!
 * It didn't!
*/

static K_SEM_DEFINE(scan,0,1);
static K_SEM_DEFINE(store,0,1);

BUILD_ASSERT(!IS_ENABLED(CONFIG_LTE_AUTO_INIT_AND_CONNECT),
	"The sample does not support automatic LTE connection establishment");

static atomic_t connected;
static K_SEM_DEFINE(lte_connected, 0, 1);
static K_SEM_DEFINE(rrc_idle, 0, 1);
static K_SEM_DEFINE(cell_data_ready, 0, 1);
#if defined(CONFIG_MULTICELL_LOCATION_SAMPLE_REQUEST_CELL_CHANGE)
static struct k_work cell_change_search_work;
#endif
#if defined(CONFIG_MULTICELL_LOCATION_SAMPLE_REQUEST_PERIODIC)
static struct k_work_delayable periodic_search_work;
#endif
static struct lte_lc_ncell neighbor_cells[CONFIG_LTE_NEIGHBOR_CELLS_MAX];
static struct lte_lc_cells_info cell_data = {
	.neighbor_cells = neighbor_cells,
};

/************
 * blink
 * 
 * 
*/
/*
void blink(void)
{
	int ret;

	if (!device_is_ready(led0.port)) {
		return;
	}

	ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return;
	}

	//blink a few times
	int count = 0;
	while (count < 5) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return;
		}
		k_msleep(SLEEP_TIME_MS);
		count++;
	}
}

void blinkTimes(int times)
{
	int ret;

	if (!device_is_ready(led.port)) 
	{
		return;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) 
	{
		return;
	}
	
	//loop here
	int loop;
	for(loop = 0; loop < times; loop++)
	{
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) 
		{
			return;
		}
		k_msleep(SLEEP_TIME_MS);
	}
}
*/

/****
 * send signals to the raspberry pi pico
*/

void Tower_Read_Success_Change(void)
{
	int ret;

	if (!device_is_ready(Tower_Read_Success.port)) 
	{
		printk("Tower Read Success Button not ready!\n");
		return;
	}

	ret = gpio_pin_configure_dt(&Tower_Read_Success, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) 
	{
		printk("Tower Read Success Button not configured!\n");
		return;
	}

	//ret = gpio_pin_toggle_dt(&Tower_Read_Success);
	//ret = gpio_pin_toggle_dt(&Tower_Read_Success);
	ret  = gpio_pin_set_dt(&Tower_Read_Success, HIGH);
	k_msleep(TOGGLE_TIME);
	ret  = gpio_pin_set_dt(&Tower_Read_Success, LOW);
	
}

void Tower_Read_Fail_Change(void)
{
	int ret;

	if (!device_is_ready(Tower_Read_Fail.port)) 
	{
		printk("Tower Read Fail Button not ready!\n");
		return;
	}

	ret = gpio_pin_configure_dt(&Tower_Read_Fail, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) 
	{
		printk("Tower Read Fail Button not configured!\n");
		return;
	}
	/*
	ret = gpio_pin_toggle_dt(&Tower_Read_Fail);
	k_msleep(SLEEP_TIME_MS);
	ret = gpio_pin_toggle_dt(&Tower_Read_Fail);
	*/
	ret  = gpio_pin_set_dt(&Tower_Read_Fail, HIGH);
	k_msleep(TOGGLE_TIME);
	ret  = gpio_pin_set_dt(&Tower_Read_Fail, LOW);
	
}

void Tower_Buffer_Full_Change(void)
{
	int ret;

	if (!device_is_ready(Tower_Buffer_Full.port)) 
	{
		printk("Tower Buffer Button not ready!\n");
		return;
	}

	ret = gpio_pin_configure_dt(&Tower_Buffer_Full, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) 
	{
		printk("Tower Buffer Button not configured!\n");
		return;
	}
	/*
	ret = gpio_pin_toggle_dt(&Tower_Buffer_Full);
	k_msleep(SLEEP_TIME_MS);
	ret = gpio_pin_toggle_dt(&Tower_Buffer_Full);
	*/
	ret  = gpio_pin_set_dt(&Tower_Buffer_Full, HIGH);
	k_msleep(TOGGLE_TIME);
	ret  = gpio_pin_set_dt(&Tower_Buffer_Full, LOW);
	
}

void Tower_Connected_Change(void)
{
	int ret;

	if (!device_is_ready(Tower_Connected.port)) 
	{
		printk("Tower Connected Button not ready!\n");
		return;
	}

	ret = gpio_pin_configure_dt(&Tower_Connected, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) 
	{
		printk("Tower Connected Button not configured!\n");
		return;
	}

	/*
	ret = gpio_pin_toggle_dt(&Tower_Connected);
	k_msleep(SLEEP_TIME_MS);
	ret = gpio_pin_toggle_dt(&Tower_Connected);
	*/
	ret  = gpio_pin_set_dt(&Tower_Connected, HIGH);
	k_msleep(TOGGLE_TIME);
	ret  = gpio_pin_set_dt(&Tower_Connected, LOW);
	
}

/*****
 * Begin Power Fucntion Section
 * This turns the board off and on again
 * It comes from the nordic + zephyr "system_off" example
 * 
*/

static int disable_ds_1(const struct device *dev)
{
	ARG_UNUSED(dev);

	pm_policy_state_lock_get(PM_STATE_SOFT_OFF, PM_ALL_SUBSTATES);
	return 0;
}

SYS_INIT(disable_ds_1, PRE_KERNEL_2, 0);

void power(void)
{
	int rc;
	const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

	if (!device_is_ready(cons)) {
		printk("%s: device not ready.\n", cons->name);
		return;
	}

	if (IS_ENABLED(CONFIG_APP_RETENTION)) {
		bool retained_ok = retained_validate();

		/* Increment for this boot attempt and update. */
		retained.boots += 1;
		retained_update();

		printk("Retained data: %s\n", retained_ok ? "valid" : "INVALID");
		printk("Boot count: %u\n", retained.boots);
		printk("Off count: %u\n", retained.off_count);
		printk("Active Ticks: %" PRIu64 "\n", retained.uptime_sum);
	} else {
		printk("Retained data not supported\n");
	}

	//NEED TO FIX THIS SWITCH!!!!!!
	
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(sw3), gpios),
			   NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(sw3), gpios),
			       NRF_GPIO_PIN_SENSE_LOW);
	
	/*
	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(Power_Button, gpios),
			   NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(Power_Button, gpios),
			       NRF_GPIO_PIN_SENSE_LOW); 
	*/	
	/*
	nrf_gpio_cfg_input(Power_Button, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_sense_set(Power_Button, NRF_GPIO_PIN_SENSE_LOW);
	*/		   
	printk("Entering system off; press BUTTON1 to restart\n");

	if (IS_ENABLED(CONFIG_APP_RETENTION)) {
		/* Update the retained state */
		retained.off_count += 1;
		retained_update();
	}

	/* Above we disabled entry to deep sleep based on duration of
	 * controlled delay.  Here we need to override that, then
	 * force entry to deep sleep on any delay.
	 */
	pm_state_force(0u, &(struct pm_state_info){PM_STATE_SOFT_OFF, 0, 0});

	/* Now we need to go sleep. This will let the idle thread runs and
	 * the pm subsystem will use the forced state. To confirm that the
	 * forced state is used, lets set the same timeout used previously.
	 */
	k_sleep(K_SECONDS(SLEEP_S));

	printk("ERROR: System off failed\n");
	while (true) {
		/* spin to avoid fall-off behavior */
	}
}

/***************************************************************************
 * Begin storage function section
 * initialize
 * list directories and files
 * append character
 * append string 
*/
void storage(void)
{
	/* raw disk i/o */
	do {
		static const char *disk_pdrv = "SD";
		uint64_t memory_size_mb;
		uint32_t block_count;
		uint32_t block_size;

		if (disk_access_init(disk_pdrv) != 0) {
			printk("Storage init ERROR!");
			break;
		}

		if (disk_access_ioctl(disk_pdrv,
				DISK_IOCTL_GET_SECTOR_COUNT, &block_count)) {
			printk("Unable to get sector count");
			break;
		}
		printk("Block count %u\n", block_count);

		if (disk_access_ioctl(disk_pdrv,
				DISK_IOCTL_GET_SECTOR_SIZE, &block_size)) {
			printk("Unable to get sector size");
			break;
		}
		printk("Sector size %u\n", block_size);

		memory_size_mb = (uint64_t)block_count * block_size;
		printk("Memory Size(MB) %u\n", (uint32_t)(memory_size_mb >> 20));
	} while (0);

	mp.mnt_point = disk_mount_pt;

	int res = fs_mount(&mp);

	if (res == FR_OK) {
		printk("Disk mounted.\n");
		//lsdir(disk_mount_pt);
	} else {
		printk("Error mounting disk.\n");
	}

	//disable infinite loop
	/*
	while (1) {
		k_sleep(K_MSEC(1000));
	}
	*/
}

static int lsdir(const char *path)
{
	int res;
	struct fs_dir_t dirp;
	static struct fs_dirent entry;

	fs_dir_t_init(&dirp);

	/* Verify fs_opendir() */
	res = fs_opendir(&dirp, path);
	if (res) {
		printk("Error opening dir %s [%d]\n", path, res);
		return res;
	}

	printk("\nListing dir %s ...\n", path);
	for (;;) {
		/* Verify fs_readdir() */
		res = fs_readdir(&dirp, &entry);

		/* entry.name[0] == 0 means end-of-dir */
		if (res || entry.name[0] == 0) {
			break;
		}

		if (entry.type == FS_DIR_ENTRY_DIR) {
			printk("[DIR] %s\n", entry.name);
		} else {
			printk("[FILE] %s (size = %zu)\n",
				entry.name, entry.size);
		}
	}

	/* Verify fs_closedir() */
	fs_closedir(&dirp);

	return res;
}

//append a single character to a file
static int AppendCharacter(const char *fname, char c)
{
	//create file object
	struct fs_file_t firp;
	fs_file_t_init(&firp);


	//set variable to hold status of file
	int fileStatus;

	//open file in append and write mode
	fileStatus = fs_open(&firp,fname,FS_O_APPEND | FS_O_WRITE);
	if(fileStatus)
    {
        printk("error open %s\n", fname);
    }

	//convert characters to valid characters
	char cc;
	if( (c >= 'a' && c <= 'z') 
		||  (c >= 'A' && c <= 'Z') 
		||  (c >= '0' && c <= '9')
		||  (c == '\n') ||  (c == '_'))
	{
		cc = c;
	}

	else
	{
		cc = '_';
	}
	
	

	//write the character to the file
	fileStatus = fs_write(&firp,&cc,1);
    if(fileStatus < 0)
    {
        printk("other error write %s\n", fname);
    }

	//check for specific errors
	else if(fileStatus == EBADF)
	{
		printk("file not opened or closed %s\n", fname);
	}
	else if(fileStatus == ENOTSUP)
	{
		printk("not implemented by underlying FS %s\n", fname);
	}

	//close the file
	fileStatus = fs_close(&firp);
    if(fileStatus)
    {
        printk("error close%s!\n", fname);
    }

return 0;
	//end
}

//append a string to a file
//if debug is true, lines are printed to the screen
void AppendString(const char *fname, char *str, int length, bool debug)
{
	//create file object
	struct fs_file_t firp;
	fs_file_t_init(&firp);
	
	if(debug == true)
	{
		printk("file object created!\n");
	}
	

	//set variable to hold status of file
	int fileStatus;

	//open file in append and write mode
	fileStatus = fs_open(&firp,fname, FS_O_APPEND | FS_O_WRITE);
	if(fileStatus)
    {
        printk("error open %s\n", fname);
    }

	if(debug == true)
	{
		printk("file opened!\n ");
	}


	if(debug == true)
	{
		printk("length: %d\n", length);
	}

	if(debug == true)
	{
		printk("copied to write string\n");
	}

	//convert string by getting rid of spaces and special characters
	char conStr[length];
	strcpy(conStr,str);

	if(debug == true)
	{
		printk("Printing string %s\n", conStr);
	}
	int loop;
	if(debug == true)
		{
			//printk("Printing characters %d", loop);
			printk("Printing characters\n");
		}
	for(loop = 0; loop < length; loop++)
	{
		
		//make sure all characters are valid
		if( (conStr[loop] >= 'a' && conStr[loop] <= 'z') 
			||  (conStr[loop] >= 'A' && conStr[loop] <= 'Z') 
			||  (conStr[loop] >= '0' && conStr[loop] <= '9')
			||  (conStr[loop] == '\n') ||  (conStr[loop] == '/') 
			||  (conStr[loop] == ':') || (conStr[loop] == '+') 
			|| (conStr[loop] == '*'))
		{
			if(debug == true)
			{
				printk("%c ", conStr[loop]);
			}
		}
		else
		{
			conStr[loop] = '_';
		}
	}

	if(debug == true)
	{
		printk("Printing string %s\n", conStr);
	}	

	//write the character to the file
	fileStatus = fs_write(&firp,&conStr,length+1);
	if(fileStatus < 0)
    {
        printk("other error write %s\n", fname);
    }

	//check for specific errors
	else if(fileStatus == EBADF)
	{
		printk("file not opened or closed %s\n", fname);
	}
	else if(fileStatus == ENOTSUP)
	{
		printk("not implemented by underlying FS %s\n", fname);
	}

	//close the file
	fileStatus = fs_close(&firp);
    if(fileStatus)
    {
        printk("error close%s!\n", fname);
    }

	if(debug == true)
	{
		printk("Done with Append String!\n");
	}
}

//empty the tower strings
void empty(void)
{
	//erase the contents of each tower
	memset(t1ID, 0, sizeof(t1ID));
	memset(t2ID, 0, sizeof(t2ID));
	memset(t3ID, 0, sizeof(t3ID));
	

	//set the towers to contain the word empty
	//printk("Setting towers to empty!\n");
	strcpy(t1ID, "empty");
	strcpy(t2ID, "empty");
	strcpy(t3ID, "empty");
	strcpy(t1COPS, "noCOPS1");
	strcpy(t2COPS, "noCOPS2");
	strcpy(t3COPS, "noCOPS3");
	strcpy(t1SNR,"noSNR1");
	strcpy(t2SNR,"noSNR2");
	strcpy(t3SNR,"noSNR3");
	printk("Towers emptied!\n");

}

/*
* Multicell Location
*
*/

static void lte_handler(const struct lte_lc_evt *const evt)
{
	switch (evt->type) {
	case LTE_LC_EVT_NW_REG_STATUS:
		if ((evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_HOME) &&
		     (evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_ROAMING)) {
			break;
		}

		LOG_INF("Network registration status: %s",
			evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME ?
			"Connected - home network" : "Connected - roaming");
		//Set tower connected pin high
		//Tower_Connected_Change();
		k_sem_give(&lte_connected);
		break;
	case LTE_LC_EVT_PSM_UPDATE:
		LOG_INF("PSM parameter update: TAU: %d, Active time: %d",
			evt->psm_cfg.tau, evt->psm_cfg.active_time);
		break;
	case LTE_LC_EVT_EDRX_UPDATE: {
		char log_buf[60];
		ssize_t len;

		len = snprintk(log_buf, sizeof(log_buf),
			       "eDRX parameter update: eDRX: %f, PTW: %f",
			       evt->edrx_cfg.edrx, evt->edrx_cfg.ptw);
		if (len > 0) {
			LOG_INF("%s", log_buf);
		}
		break;
	}
	case LTE_LC_EVT_RRC_UPDATE:
		LOG_INF("RRC mode: %s",
			evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED ?
			"Connected" : "Idle");
		if (evt->rrc_mode == LTE_LC_RRC_MODE_IDLE) {
			k_sem_give(&rrc_idle);
		} else {
			k_sem_take(&rrc_idle, K_NO_WAIT);
		}
		break;
	case LTE_LC_EVT_CELL_UPDATE: {
		LOG_INF("LTE cell changed: Cell ID: %d, Tracking area: %d",
			evt->cell.id, evt->cell.tac);

#if defined(CONFIG_MULTICELL_LOCATION_SAMPLE_REQUEST_CELL_CHANGE)
		static uint32_t prev_cell_id;

		if (evt->cell.id != prev_cell_id) {
			if (evt->cell.id != LTE_LC_CELL_EUTRAN_ID_INVALID) {
				k_work_submit(&cell_change_search_work);
			}

			prev_cell_id = evt->cell.id;
		}
#endif
		break;
	}
	case LTE_LC_EVT_LTE_MODE_UPDATE:
		LOG_INF("Active LTE mode changed: %s",
			evt->lte_mode == LTE_LC_LTE_MODE_NONE ? "None" :
			evt->lte_mode == LTE_LC_LTE_MODE_LTEM ? "LTE-M" :
			evt->lte_mode == LTE_LC_LTE_MODE_NBIOT ? "NB-IoT" :
			"Unknown");
		break;
	case LTE_LC_EVT_NEIGHBOR_CELL_MEAS:
		LOG_INF("Neighbor cell measurements received");

		if (evt->cells_info.current_cell.id == LTE_LC_CELL_EUTRAN_ID_INVALID) {
			LOG_DBG("Cell ID not valid.");
			break;
		}

		/* Copy current cell information. */
		memcpy(&cell_data.current_cell,
		       &evt->cells_info.current_cell,
		       sizeof(struct lte_lc_cell));

		/* Copy neighbor cell information if present. */
		
		if (evt->cells_info.ncells_count > 0 && evt->cells_info.neighbor_cells) {
			memcpy(neighbor_cells,
			       evt->cells_info.neighbor_cells,
			       sizeof(struct lte_lc_ncell) * evt->cells_info.ncells_count);

			cell_data.ncells_count = evt->cells_info.ncells_count;
		} else {
			cell_data.ncells_count = 0;
		}
		

		LOG_INF("Neighbor cells found: %d", cell_data.ncells_count);

		k_sem_give(&cell_data_ready);
		break;
	default:
		break;
	}

	
}

static int lte_connect(void)
{
	int err;

	if (IS_ENABLED(CONFIG_MULTICELL_LOCATION_SAMPLE_PSM)) {
		err = lte_lc_psm_req(true);
		if (err) {
			LOG_ERR("Failed to request PSM, error: %d", err);
		}
	} else {
		err = lte_lc_psm_req(false);
		if (err) {
			LOG_ERR("Failed to disable PSM, error: %d", err);
		}
	}

	if (IS_ENABLED(CONFIG_MULTICELL_LOCATION_SAMPLE_EDRX)) {
		err = lte_lc_edrx_req(true);
		if (err) {
			LOG_ERR("Failed to request eDRX, error: %d", err);
		}
	} else {
		err = lte_lc_edrx_req(false);
		if (err) {
			LOG_ERR("Failed to disable eDRX, error: %d", err);
		}
	}

	err = lte_lc_init_and_connect_async(lte_handler);
	if (err) {
		LOG_ERR("Modem could not be configured, error: %d",
			err);
		return err;
	}

	/* Check LTE events of type LTE_LC_EVT_NW_REG_STATUS in
	 * lte_handler() to determine when the LTE link is up.
	 */

	return 0;
}

//change COPS
void changeCOPS(char *opName, int lenOpName)
{
	int err;
    char response[64];

	char cmd[25] = "AT+COPS=1,2,\"";

	//printk("new COPS: %s\n", opName);

	strcat(cmd,opName);
	strcat(cmd,"\"");

	// err = nrf_modem_at_cmd(response, sizeof(response), "AT+COPS=1,2,\"310410\"");
    err = nrf_modem_at_cmd(response, sizeof(response), cmd);
	if (err) 
	{
        //error
    }
    printk("Modem response:\n%s", response);
}

static void start_cell_measurements(void)
{
	int err;

	if (!atomic_get(&connected)) {
		return;
	}

	err = lte_lc_neighbor_cell_measurement(NULL);
	if (err) {
		LOG_ERR("Failed to initiate neighbor cell measurements, error: %d",
			err);
	}
}

/*
static void button_handler(uint32_t button_states, uint32_t has_changed)
{
	if (has_changed & button_states & DK_BTN1_MSK) {
		if (!atomic_get(&connected)) {
			LOG_INF("Ignoring button press, not connected to network");
			return;
		}

		LOG_INF("Button 1 pressed, starting cell measurements");
		start_cell_measurements();
	}
}
*/

#if defined(CONFIG_MULTICELL_LOCATION_SAMPLE_REQUEST_CELL_CHANGE)
static void cell_change_search_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!atomic_get(&connected)) {
		return;
	}

	LOG_INF("Cell change triggered start of cell measurements");
	start_cell_measurements();
}
#endif

#if defined(CONFIG_MULTICELL_LOCATION_SAMPLE_REQUEST_PERIODIC)
static void periodic_search_work_fn(struct k_work *work)
{
	LOG_INF("Periodical start of cell measurements");
	start_cell_measurements();

	k_work_reschedule(k_work_delayable_from_work(work),
		K_SECONDS(CONFIG_MULTICELL_LOCATION_SAMPLE_REQUEST_PERIODIC_INTERVAL));
}
#endif

void getSNR(int towerNum)
{
	int err;
    char response[towerSizeSNR];

	err = nrf_modem_at_cmd(response, sizeof(response), "AT%%CONEVAL");
    if (err) 
	{
        //error
    }

	int dataSize = towerSizeSNR;
	char data[dataSize];
	strcpy(data,response);
	int data_length = strlen(data);

	int loop;
		for(loop = 0; loop < data_length; loop++)
		{
		
			//only want A-Z, a-z, 0-9, and newline characters
			if( (data[loop] >= 'a' && data[loop] <= 'z') 
			||  (data[loop] >= 'A' && data[loop] <= 'Z') 
			||  (data[loop] >= '0' && data[loop] <= '9')
			||  (data[loop] == '\n') ||  (data[loop] == '/') 
			||  (data[loop] == ':') || (data[loop] == '+'))
			{
				//printk("%c", data[loop]);
			}
			//set all other characters to underscore
			else
			{
				data[loop] = '_';
			}
		}

		//printk("SNR: %s", data);

		//if((strcmp(t1ID, "empty") == 0))
		if(towerNum == 1)
		{
			strcpy(t1SNR, data);
		}	
		//else if((strcmp(t2ID, "empty") == 0))
		else if(towerNum == 2)
		{
			strcpy(t2SNR, data);
		}
		//else if((strcmp(t3ID, "empty") == 0))
		else if(towerNum == 3)
		{
			strcpy(t3SNR, data);
		}
		else
		{
			printk("Not stored!\n");
		}

    //printk("Modem response:\n%s", response);
}

static void print_cell_data(void)
{
	if (cell_data.current_cell.id == LTE_LC_CELL_EUTRAN_ID_INVALID) {
		LOG_WRN("No cells were found");
		//Set tower read fail pin high
		Tower_Read_Fail_Change();
		return;
	}
	printk("Current cell:\n");
	printk("\tMCC: %03d\n", cell_data.current_cell.mcc);
	printk("\tMNC: %03d\n", cell_data.current_cell.mnc);
	printk("\tCell ID: %d\n", cell_data.current_cell.id);
	printk("\tTAC: %d\n", cell_data.current_cell.tac);
	printk("\tEARFCN: %d\n", cell_data.current_cell.earfcn);
	printk("\tTiming advance: %d\n", cell_data.current_cell.timing_advance);
	printk("\tMeasurement time: %lld\n", cell_data.current_cell.measurement_time);
	printk("\tPhysical cell ID: %d\n", cell_data.current_cell.phys_cell_id);
	printk("\tRSRP: %d\n", cell_data.current_cell.rsrp);
	printk("\tRSRQ: %d\n", cell_data.current_cell.rsrq);
	

	//check if towers are empty and unique
	char newID[20];
	sprintf(newID,"%d",cell_data.current_cell.id);
	char newTA[20];
	sprintf(newTA,"%d",cell_data.current_cell.timing_advance);
	char newCOPS[25];
	sprintf(newCOPS,"%03d",cell_data.current_cell.mcc);
	char newMNC[13];
	sprintf(newMNC,"%03d",cell_data.current_cell.mnc);
	strcat(newCOPS,newMNC);
	
	//make sure timing advance is a valid number
	if(cell_data.current_cell.timing_advance != 65535)
	{
		if((strstr(t1ID, "empty") != NULL) ) //&& (strstr(newTA, "65535") != NULL)
		{	
			//strcpy(t1ID,cell_data.current_cell.id);
			sprintf(t1ID,"%d",cell_data.current_cell.id);
			//AppendString(testTxt, newID, strlen(newID),false);
			//AppendCharacter(testTxt, '\n');
			sprintf(t1MCC,"%03d",cell_data.current_cell.mcc);
			sprintf(t1MNC,"%03d",cell_data.current_cell.mnc);
			sprintf(t1TAC,"%d",cell_data.current_cell.tac);
			sprintf(t1TA,"%d",cell_data.current_cell.timing_advance);
			sprintf(t1RSRP,"%d",cell_data.current_cell.rsrp);
			sprintf(t1RSRQ,"%d",cell_data.current_cell.rsrq);
			strcpy(t1COPS,t1MCC);
			strcat(t1COPS,t1MNC);

			//get the connection value
			//getSNR(1);
		}

		//else if((strstr(t2ID, "empty") != NULL) && (!(strstr(t1ID, newID) != NULL) || !(strstr(t1COPS, newCOPS) != NULL) )) // && !(strstr(newTA, "65535") != NULL)
		else if((strstr(t2ID, "empty") != NULL) && (!(strstr(t1ID, newID) != NULL)))
		{	
			//strcpy(t2ID,cell_data.current_cell.id);
			sprintf(t2ID,"%d",cell_data.current_cell.id);
			sprintf(t2MCC,"%03d",cell_data.current_cell.mcc);
			sprintf(t2MNC,"%03d",cell_data.current_cell.mnc);
			sprintf(t2TAC,"%d",cell_data.current_cell.tac);
			sprintf(t2TA,"%d",cell_data.current_cell.timing_advance);
			sprintf(t2RSRP,"%d",cell_data.current_cell.rsrp);
			sprintf(t2RSRQ,"%d",cell_data.current_cell.rsrq);
			strcpy(t2COPS,t2MCC);
			strcat(t2COPS,t2MNC);

			//getSNR(2);
		}

		//else if((strstr(t3ID, "empty") != NULL) && (!(strstr(t2ID, newID) != NULL) || !(strstr(t2COPS, newCOPS) != NULL) ) && (!(strstr(t1ID, newID) != NULL) || !(strstr(t1COPS, newCOPS) != NULL) ) ) //&& !(strstr(newTA, "65535") != NULL)
		else if((strstr(t3ID, "empty") != NULL) && (!(strstr(t2ID, newID) != NULL)) && (!(strstr(t1ID, newID) != NULL)))
		{	
			//strcpy(t3ID,cell_data.current_cell.id);
			sprintf(t3ID,"%d",cell_data.current_cell.id);
			sprintf(t3MCC,"%03d",cell_data.current_cell.mcc);
			sprintf(t3MNC,"%03d",cell_data.current_cell.mnc);
			sprintf(t3TAC,"%d",cell_data.current_cell.tac);
			sprintf(t3TA,"%d",cell_data.current_cell.timing_advance);
			sprintf(t3RSRP,"%d",cell_data.current_cell.rsrp);
			sprintf(t3RSRQ,"%d",cell_data.current_cell.rsrq);
			strcpy(t3COPS,t3MCC);
			strcat(t3COPS,t3MNC);

			//getSNR(3);
		}

		else
		{
			printk("Information not recorded!\n");
			//Set tower read fail pin high
			Tower_Read_Fail_Change();
		}
	}

	else
	{
		printk("Invalid timing advance!\n");
		//Set tower read fail pin high
		Tower_Read_Fail_Change();
	}

	

	printk("Currently stored towers:\n");
	printk("Tower 1: %s\n", t1ID);
	printk("Tower 2: %s\n", t2ID);
	printk("Tower 3: %s\n", t3ID);

	printk("Tower 1 COPS: %s\n", t1COPS);
	printk("Tower 2 COPS: %s\n", t2COPS);
	printk("Tower 3 COPS: %s\n", t3COPS);

	//set the tower buffer to full
	if(!(strstr(t1ID, "empty") != NULL) && !(strstr(t2ID, "empty") != NULL) && !(strstr(t3ID, "empty") != NULL))
	{
		Tower_Buffer_Full_Change();
	}

	

	//set the value of COPS

	//check if 1st tower is t-mobile
	if(strstr(t1COPS, t_mobile_cops) != NULL)
	{
		//check if 2nd tower is at&t
		if(strstr(t2COPS, att_mobility_cops) != NULL)
		{
			//run cops firstnet
			//changeCOPS(firstnet_cops, strlen(firstnet_cops));
			changeCOPS(verizon, strlen(verizon));
		}
		else
		{
			//run at&t
			changeCOPS(att_mobility_cops, strlen(att_mobility_cops));
		}
	}

	//check if first tower is at&t
	else if(strstr(t1COPS, att_mobility_cops) != NULL)
	{
		//check if 2nd tower is t-mobile
		if(strstr(t2COPS, t_mobile_cops) != NULL)
		{
			//run cops firstnet
			//changeCOPS(firstnet_cops, strlen(firstnet_cops));
			changeCOPS(verizon, strlen(verizon));
		}
		else
		{
			//run t-mobile
			changeCOPS(t_mobile_cops, strlen(t_mobile_cops));
		}
	}

	//check if first tower is firstnet
	//else if(strstr(t1COPS, firstnet_cops) != NULL)
	else if(strstr(t1COPS, verizon) != NULL)
	{
		//check if 2nd tower is t-mobile
		if(strstr(t2COPS, t_mobile_cops) != NULL)
		{
			//run cops firstnet
			changeCOPS(att_mobility_cops, strlen(att_mobility_cops));
		}
		else
		{
			//run at&t
			changeCOPS(t_mobile_cops, strlen(t_mobile_cops));
		}
	}
	else
	{
		changeCOPS(t_mobile_cops, strlen(t_mobile_cops));
	}
}

void getTime(void)
{
	int err;
    char response[64];

	err = nrf_modem_at_cmd(response, sizeof(response), "AT+CCLK?");
    if (err) 
	{
        //error
    }

	int dataSize = 64;
	char data[dataSize];
	strcpy(data,response);
	int data_length = strlen(data);

	int loop;
		for(loop = 0; loop < data_length; loop++)
		{
		
			//only want A-Z, a-z, 0-9, and newline characters
			if( (data[loop] >= 'a' && data[loop] <= 'z') 
			||  (data[loop] >= 'A' && data[loop] <= 'Z') 
			||  (data[loop] >= '0' && data[loop] <= '9')
			||  (data[loop] == '\n') ||  (data[loop] == '/') 
			||  (data[loop] == ':') || (data[loop] == '+'))
			{
				//printk("%c", data[loop]);
			}
			//set all other characters to underscore
			else
			{
				data[loop] = '_';
			}
		}

		strcpy(time,data);	

    //printk("Modem response:\n%s", response);
}




/*****
 * button
 * 
 * 
*/

//custom button handler
static void button_handler(uint32_t button_states, uint32_t has_changed)
{
	//this runs if the first button on the dev kit is pressed
	//typically used for querying and printing values
	//if (has_changed & button_states & DK_BTN1_MSK) 
	if (has_changed & button_states & Collect_Towers) 
	{
		printk("button 1 \n");
		//blinkTimes(5);
		//lsdir(disk_mount_pt);
		//cfun_q();
		
		if (!atomic_get(&connected)) 
		{
			LOG_INF("Ignoring button press, not connected to network");
			//Set tower read fail pin high
			Tower_Read_Fail_Change();
			return;
		}

		LOG_INF("Button 1 pressed, starting cell measurements");
		start_cell_measurements();
	
		//readCOPS();
		//testCOPS();
	}

	//this runs if the second button on the dev kit is pressed
	//stores collected towers to disk
	if (has_changed & button_states & DK_BTN2_MSK) 
	//if (has_changed & button_states & Store_Towers)
	{
		printk("button 2 \n");
		//print_cell_data();
		//blinkTimes(5);


		
		//set and get time
		//int64_t unix_time_ms;
		//date_time_update();
		//int err= date_time_now(&unix_time_ms);
		//printk("Date_time: %i %i\n", (uint32_t)(unix_time_ms/1000000), (uint32_t)(unix_time_ms%1000000));
		//AppendString(testTxt, "hello\n", 6, true);
		//cmd_read();
		//AppendString(testTxt, t1, strlen(t1), true);
		//AppendString(testTxt, t2, strlen(t2), true);
		//AppendString(testTxt, t3, strlen(t3), true);
		printk("Tower 1: %s\n", t1ID);
		printk("Tower 2: %s\n", t2ID);
		printk("Tower 3: %s\n", t3ID);
		/*
		AppendString(testTxt, t1ID, strlen(t1ID), true);
		AppendString(testTxt, t2ID, strlen(t2ID), true);
		AppendString(testTxt, t3ID, strlen(t3ID), true);
		*/
		if((strcmp(t1ID, "empty") == 0))
		{
			strcpy(t1ID, "empty\n");
		}

		if((strcmp(t2ID, "empty") == 0))
		{
			strcpy(t2ID, "empty\n");
		}

		if((strcmp(t3ID, "empty") == 0))
		{
			strcpy(t3ID, "empty\n");
		}

		getTime();


		AppendString(testTxt, separator, strlen(separator), false);
		//tower 1
		AppendString(testTxt, t1ID, strlen(t1ID), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t1MCC, strlen(t1MCC), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t1MNC, strlen(t1MNC), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t1TAC, strlen(t1TAC), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t1TA, strlen(t1TA), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t1RSRP, strlen(t1RSRP), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t1RSRQ, strlen(t1RSRQ), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t1SNR, strlen(t1SNR), false);
		AppendCharacter(testTxt,'\n');
		//tower 2
		AppendString(testTxt, t2ID, strlen(t2ID), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t2MCC, strlen(t2MCC), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t2MNC, strlen(t2MNC), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t2TAC, strlen(t2TAC), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t2TA, strlen(t2TA), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t2RSRP, strlen(t2RSRP), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t2RSRQ, strlen(t2RSRQ), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t2SNR, strlen(t2SNR), false);
		AppendCharacter(testTxt,'\n');
		//tower 3
		AppendString(testTxt, t3ID, strlen(t3ID), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t3MCC, strlen(t3MCC), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t3MNC, strlen(t3MNC), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t3TAC, strlen(t3TAC), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t3TA, strlen(t3TA), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t3RSRP, strlen(t3RSRP), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t3RSRQ, strlen(t3RSRQ), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t3SNR, strlen(t3SNR), false);
		AppendCharacter(testTxt,'\n');
		
		AppendString(testTxt, time, strlen(time), false);
		AppendString(testTxt, separator, strlen(separator), false);
		
		empty();

		power();
	}

	if (has_changed & button_states & Output_Other) 
	{
		//printk("Unused!\n");
		printk("button 3 \n");

		/*

		printk("Tower 1: %s\n", t1ID);
		printk("Tower 2: %s\n", t2ID);
		printk("Tower 3: %s\n", t3ID);

		if((strcmp(t1ID, "empty") == 0))
		{
			strcpy(t1ID, "empty\n");
		}

		if((strcmp(t2ID, "empty") == 0))
		{
			strcpy(t2ID, "empty\n");
		}

		if((strcmp(t3ID, "empty") == 0))
		{
			strcpy(t3ID, "empty\n");
		}

		getTime();


		AppendString(testTxt, separator, strlen(separator), false);
		//tower 1
		AppendString(testTxt, t1ID, strlen(t1ID), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t1MCC, strlen(t1MCC), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t1MNC, strlen(t1MNC), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t1TAC, strlen(t1TAC), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t1TA, strlen(t1TA), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t1RSRP, strlen(t1RSRP), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t1RSRQ, strlen(t1RSRQ), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t1SNR, strlen(t1SNR), false);
		AppendCharacter(testTxt,'\n');
		//tower 2
		AppendString(testTxt, t2ID, strlen(t2ID), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t2MCC, strlen(t2MCC), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t2MNC, strlen(t2MNC), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t2TAC, strlen(t2TAC), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t2TA, strlen(t2TA), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t2RSRP, strlen(t2RSRP), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t2RSRQ, strlen(t2RSRQ), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t2SNR, strlen(t2SNR), false);
		AppendCharacter(testTxt,'\n');
		//tower 3
		AppendString(testTxt, t3ID, strlen(t3ID), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t3MCC, strlen(t3MCC), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t3MNC, strlen(t3MNC), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t3TAC, strlen(t3TAC), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t3TA, strlen(t3TA), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t3RSRP, strlen(t3RSRP), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t3RSRQ, strlen(t3RSRQ), false);
		AppendCharacter(testTxt,'\n');
		AppendString(testTxt, t3SNR, strlen(t3SNR), false);
		AppendCharacter(testTxt,'\n');
		
		AppendString(testTxt, time, strlen(time), false);
		AppendString(testTxt, separator, strlen(separator), false);
		
		empty();

		power();
		*/

	}

	//This will be the power button
	//if (has_changed & button_states & Power_Button)
	/*
	if (has_changed & button_states & DK_BTN3_MSK) 
	{
		power();
		printk("power!\n");
	}
	*/
	
}

void button1Action(void)
{
	printk("button 1 \n");
	//blinkTimes(5);
	lsdir(disk_mount_pt);
	//cfun_q();
		
	if (!atomic_get(&connected)) 
	{
		LOG_INF("Ignoring button press, not connected to network");
		return;
	}

	LOG_INF("Button 1 pressed, starting cell measurements");
	start_cell_measurements();
	return;
}

void button2Action(void)
{
	printk("button 2 \n");
	//print_cell_data();
	//blinkTimes(5);
	printk("Tower 1: %s\n", t1ID);
	printk("Tower 2: %s\n", t2ID);
	printk("Tower 3: %s\n", t3ID);
	if((strcmp(t1ID, "empty") == 0))
	{
		strcpy(t1ID, "empty\n");
	}

	if((strcmp(t2ID, "empty") == 0))
	{
		strcpy(t2ID, "empty\n");
	}

	if((strcmp(t3ID, "empty") == 0))
	{
		strcpy(t3ID, "empty\n");
	}

	getTime();


	AppendString(testTxt, separator, strlen(separator), false);
	//tower 1
	AppendString(testTxt, t1ID, strlen(t1ID), false);
	AppendCharacter(testTxt,'\n');
	AppendString(testTxt, t1MCC, strlen(t1MCC), false);
	AppendCharacter(testTxt,'\n');
	AppendString(testTxt, t1MNC, strlen(t1MNC), false);
	AppendCharacter(testTxt,'\n');
	AppendString(testTxt, t1TAC, strlen(t1TAC), false);
	AppendCharacter(testTxt,'\n');
	AppendString(testTxt, t1TA, strlen(t1TA), false);
	AppendCharacter(testTxt,'\n');
	//tower 2
	AppendString(testTxt, t2ID, strlen(t2ID), false);
	AppendCharacter(testTxt,'\n');
	AppendString(testTxt, t2MCC, strlen(t2MCC), false);
	AppendCharacter(testTxt,'\n');
	AppendString(testTxt, t2MNC, strlen(t2MNC), false);
	AppendCharacter(testTxt,'\n');
	AppendString(testTxt, t2TAC, strlen(t2TAC), false);
	AppendCharacter(testTxt,'\n');
	AppendString(testTxt, t2TA, strlen(t2TA), false);
	AppendCharacter(testTxt,'\n');
	//tower 3
	AppendString(testTxt, t3ID, strlen(t3ID), false);
	AppendCharacter(testTxt,'\n');
	AppendString(testTxt, t3MCC, strlen(t3MCC), false);
	AppendCharacter(testTxt,'\n');
	AppendString(testTxt, t3MNC, strlen(t3MNC), false);
	AppendCharacter(testTxt,'\n');
	AppendString(testTxt, t3TAC, strlen(t3TAC), false);
	AppendCharacter(testTxt,'\n');
	AppendString(testTxt, t3TA, strlen(t3TA), false);
	AppendCharacter(testTxt,'\n');
	
	AppendString(testTxt, time, strlen(time), false);
	AppendString(testTxt, separator, strlen(separator), false);
		
	empty();


	return;
}

void main(void)
{
	//initialize storage
	//blinkTimes(1);
	storage();
	
	int startFakeLEDs;
	startFakeLEDs = gpio_pin_set_dt(&Tower_Read_Success, LOW);
	startFakeLEDs = gpio_pin_set_dt(&Tower_Read_Fail, LOW);
	startFakeLEDs = gpio_pin_set_dt(&Tower_Buffer_Full, LOW);
	startFakeLEDs = gpio_pin_set_dt(&Tower_Connected, LOW);

	//list all files and folders
	lsdir(disk_mount_pt);
	//blinkTimes(2);

	//initialize global character arrays to contain the word empty
	empty();
	//blinkTimes(3);

	//set the tower cops
	strcpy(t1COPS, "noCOPS1");
	strcpy(t2COPS, "noCOPS2");
	strcpy(t3COPS, "noCOPS3");
	
	int err;

	//LOG_INF("Multicell location sample has started");
	LOG_INF("Main has started!");

#if defined(CONFIG_MULTICELL_LOCATION_SAMPLE_REQUEST_CELL_CHANGE)
	k_work_init(&cell_change_search_work, cell_change_search_work_fn);
#endif
#if defined(CONFIG_MULTICELL_LOCATION_SAMPLE_REQUEST_PERIODIC)
	k_work_init_delayable(&periodic_search_work, periodic_search_work_fn);
#endif

	err = multicell_location_provision_certificate(false);
	if (err) {
		LOG_ERR("Certificate provisioning failed, exiting application");
		return;
	}

	err = dk_buttons_init(button_handler);
	if (err) {
		LOG_ERR("Failed to initialize DK library, error: %d", err);
	}

	err = lte_connect();
	if (err) {
		LOG_ERR("Failed to connect to LTE network, error: %d", err);
		return;
	}

	LOG_INF("Connecting to LTE network, this may take several minutes...");

	k_sem_take(&lte_connected, K_FOREVER);
	atomic_set(&connected, 1);

	LOG_INF("Connected to LTE network");
	//Set tower connected pin high
	//Tower_Connected_Change();

	/* Wait until RRC connection has been released, otherwise modem will not be able to
	 * measure neighbor cells unless currently configured by the network.
	 */
	k_sem_take(&rrc_idle, K_FOREVER);

#if defined(CONFIG_MULTICELL_LOCATION_SAMPLE_REQUEST_PERIODIC)
	LOG_INF("Requesting neighbor cell information every %d seconds",
		CONFIG_MULTICELL_LOCATION_SAMPLE_REQUEST_PERIODIC_INTERVAL);
	k_work_schedule(&periodic_search_work, K_NO_WAIT);
#else
	start_cell_measurements();
#endif

	//printk("About to enter loop!\n");
	k_timeout_t timeout = K_MSEC(1000);
	//k_sem_give(&scan);
	int getData = 0;
	int loopForever;
	for(loopForever = 0; loopForever < 2; )
	{
		start:
		k_sem_take(&cell_data_ready, K_FOREVER);

		if (CONFIG_MULTICELL_LOCATION_SAMPLE_PRINT_DATA) 
		{
			//printk("Now printing cell data!\n");
			//print_cell_data();
		}

		//get the scan lock
		/*
		if(k_sem_take(&scan, timeout) != 0)
		{
			printk("Scan is not available!\n");
			print_cell_data();
			printk("scan: %d\n", k_sem_count_get(&scan));
			k_sem_give(&scan);
			goto start;
		}
		else
		{
			printk("Data is being gathered!\n");
			start_cell_measurements();
			printk("scan: %d\n", k_sem_count_get(&scan));
			k_sem_take(&scan, timeout);
			goto start;
		}
		*/
		//k_msleep(1000);

		
		if(getData == 1)
		{
			printk("Scan is not available!\n");
			print_cell_data();
			//printk("scan: %d\n", k_sem_count_get(&scan));
			getData = 0;
			//goto start;
		}
		else
		{
			printk("Data is being gathered!\n");
			if (!atomic_get(&connected)) 
			{
				LOG_INF("Ignoring button press, not connected to network");
				goto start;
			}
			start_cell_measurements();
			//printk("scan: %d\n", k_sem_count_get(&scan));
			getData = 1;
			//goto start;
		}
		

	}
}
