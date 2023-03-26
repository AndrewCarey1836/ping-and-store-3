/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/* 
* Modified by Andrew Carey
*/

#include <zephyr/kernel.h>
#include <stdio.h>
#include <modem/lte_lc.h>
#include <dk_buttons_and_leds.h>
#include <net/multicell_location.h>

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
 * set up for blink
*/
#define SLEEP_TIME_MS   100
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

//towers as a global string
//#define towerSize 200
#define towerSizeID 15
#define towerSizeMCC 12
#define towerSizeMNC 12
#define towerSizeTAC 12
#define towerSizeTA 12

//towers 

//tower 1
char t1ID[towerSizeID];
char t1MCC[towerSizeMCC];
char t1MNC[towerSizeMNC];
char t1TAC[towerSizeTAC];
char t1TA[towerSizeTA];

//tower 2
char t2ID[towerSizeID];
char t2MCC[towerSizeMCC];
char t2MNC[towerSizeMNC];
char t2TAC[towerSizeTAC];
char t2TA[towerSizeTA];

//tower 3
char t3ID[towerSizeID];
char t3MCC[towerSizeMCC];
char t3MNC[towerSizeMNC];
char t3TAC[towerSizeTAC];
char t3TA[towerSizeTA];


char time[towerSizeID];

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
void blink(void)
{
	int ret;

	if (!device_is_ready(led.port)) {
		return;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
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
	printk("Towers emptied!\n");

}


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

static void print_cell_data(void)
{
	if (cell_data.current_cell.id == LTE_LC_CELL_EUTRAN_ID_INVALID) {
		LOG_WRN("No cells were found");
		return;
	}

	LOG_INF("Current cell:");
	LOG_INF("\tMCC: %03d", cell_data.current_cell.mcc);
	LOG_INF("\tMNC: %03d", cell_data.current_cell.mnc);
	LOG_INF("\tCell ID: %d", cell_data.current_cell.id);
	LOG_INF("\tTAC: %d", cell_data.current_cell.tac);
	LOG_INF("\tEARFCN: %d", cell_data.current_cell.earfcn);
	LOG_INF("\tTiming advance: %d", cell_data.current_cell.timing_advance);
	LOG_INF("\tMeasurement time: %lld", cell_data.current_cell.measurement_time);
	LOG_INF("\tPhysical cell ID: %d", cell_data.current_cell.phys_cell_id);
	LOG_INF("\tRSRP: %d", cell_data.current_cell.rsrp);
	LOG_INF("\tRSRQ: %d", cell_data.current_cell.rsrq);

	//check if towers are empty and unique
	char newID[20];
	sprintf(newID,"%d",cell_data.current_cell.id);
	char newTA[20];
	sprintf(newTA,"%d",cell_data.current_cell.timing_advance);
	
		if((strstr(t1ID, "empty") != NULL) && !(strcmp(newTA, "65535") == 0))
		{	
			//strcpy(t1ID,cell_data.current_cell.id);
			sprintf(t1ID,"%d",cell_data.current_cell.id);
			sprintf(t1MCC,"%d",cell_data.current_cell.mcc);
			sprintf(t1MNC,"%d",cell_data.current_cell.mnc);
			sprintf(t1TAC,"%d",cell_data.current_cell.tac);
			sprintf(t1TA,"%d",cell_data.current_cell.timing_advance);
		}

		else if((strstr(t2ID, "empty") != NULL) && !(strstr(t1ID, newID) != NULL) && !(strcmp(newTA, "65535") == 0))
		{	
			//strcpy(t2ID,cell_data.current_cell.id);
			sprintf(t2ID,"%d",cell_data.current_cell.id);
			sprintf(t2MCC,"%d",cell_data.current_cell.mcc);
			sprintf(t2MNC,"%d",cell_data.current_cell.mnc);
			sprintf(t2TAC,"%d",cell_data.current_cell.tac);
			sprintf(t2TA,"%d",cell_data.current_cell.timing_advance);
		}

		else if((strstr(t3ID, "empty") != NULL) && !(strstr(t2ID, newID) != NULL) && !(strstr(t1ID, newID) != NULL) && !(strcmp(newTA, "65535") == 0))
		{	
			//strcpy(t3ID,cell_data.current_cell.id);
			sprintf(t3ID,"%d",cell_data.current_cell.id);
			sprintf(t3MCC,"%d",cell_data.current_cell.mcc);
			sprintf(t3MNC,"%d",cell_data.current_cell.mnc);
			sprintf(t3TAC,"%d",cell_data.current_cell.tac);
			sprintf(t3TA,"%d",cell_data.current_cell.timing_advance);
		}

		else
		{
			printk("Information not recorded!\n");
		}



/*
	if (cell_data.ncells_count == 0) {
		LOG_INF("*** No neighbor cells found ***");
		return;
	}

	for (size_t i = 0; i < cell_data.ncells_count; i++) {
		LOG_INF("Neighbor cell %d", i + 1);
		LOG_INF("\tEARFCN: %d", cell_data.neighbor_cells[i].earfcn);
		LOG_INF("\tTime difference: %d", cell_data.neighbor_cells[i].time_diff);
		LOG_INF("\tPhysical cell ID: %d", cell_data.neighbor_cells[i].phys_cell_id);
		LOG_INF("\tRSRP: %d", cell_data.neighbor_cells[i].rsrp);
		LOG_INF("\tRSRQ: %d", cell_data.neighbor_cells[i].rsrq);
	}
*/
}

/*
static void request_location(enum multicell_service service, const char *service_str)
{
	int err;
	struct multicell_location_params params = { 0 };
	struct multicell_location location;

	LOG_INF("Sending location request for %s ...", service_str);

	params.service = service;
	params.cell_data = &cell_data;
	params.timeout = SYS_FOREVER_MS;
	err = multicell_location_get(&params, &location);
	if (err) {
		LOG_ERR("Failed to acquire location, error: %d", err);
		return;
	}

	LOG_INF("Location obtained from %s: ", service_str);
	LOG_INF("\tLatitude: %f", location.latitude);
	LOG_INF("\tLongitude: %f", location.longitude);
	LOG_INF("\tAccuracy: %.0f", location.accuracy);
}
*/

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
	if (has_changed & button_states & DK_BTN1_MSK) 
	{
		printk("button 1 \n");
		blinkTimes(5);
		lsdir(disk_mount_pt);
		//cfun_q();
		
		if (!atomic_get(&connected)) 
		{
			LOG_INF("Ignoring button press, not connected to network");
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
	{
		printk("button 2 \n");
		print_cell_data();
		blinkTimes(5);
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

		//getTime();


		AppendString(testTxt, separator, strlen(separator), false);
		//tower 1
		AppendString(testTxt, t1ID, strlen(t1ID), false);
		AppendString(testTxt, t1MCC, strlen(t1MCC), false);
		AppendString(testTxt, t1MNC, strlen(t1MNC), false);
		AppendString(testTxt, t1TAC, strlen(t1TAC), false);
		AppendString(testTxt, t1TA, strlen(t1TA), false);
		//tower 2
		AppendString(testTxt, t2ID, strlen(t2ID), false);
		AppendString(testTxt, t2MCC, strlen(t2MCC), false);
		AppendString(testTxt, t2MNC, strlen(t2MNC), false);
		AppendString(testTxt, t2TAC, strlen(t2TAC), false);
		AppendString(testTxt, t2TA, strlen(t2TA), false);
		//tower 3
		AppendString(testTxt, t3ID, strlen(t3ID), false);
		AppendString(testTxt, t3MCC, strlen(t3MCC), false);
		AppendString(testTxt, t3MNC, strlen(t3MNC), false);
		AppendString(testTxt, t3TAC, strlen(t3TAC), false);
		AppendString(testTxt, t3TA, strlen(t3TA), false);
		//AppendString(testTxt, time, strlen(time), false);
		AppendString(testTxt, separator, strlen(separator), false);
		
		empty();
	}
}

void main(void)
{
	//initialize storage
	blinkTimes(1);
	storage();
	

	//list all files and folders
	lsdir(disk_mount_pt);
	blinkTimes(2);

	//initialize global character arrays to contain the word empty
	empty();
	blinkTimes(3);
	
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

	while (true) {
		k_sem_take(&cell_data_ready, K_FOREVER);

		if (CONFIG_MULTICELL_LOCATION_SAMPLE_PRINT_DATA) {
			print_cell_data();
		}

		/* Request location for all different services to demonstrate the possibilities */
/*
#if defined(CONFIG_MULTICELL_LOCATION_SERVICE_NRF_CLOUD)
		request_location(MULTICELL_SERVICE_NRF_CLOUD, "nRF Cloud");
#endif
#if defined(CONFIG_MULTICELL_LOCATION_SERVICE_HERE)
		request_location(MULTICELL_SERVICE_HERE, "HERE");
#endif
		request_location(MULTICELL_SERVICE_ANY, "Any");
		*/
	}
}
