#ifndef WIFI_H
#define WIFI_H
void wait_for_wifi(void);
void wifi_init_sta(void);
static void wifi_scan_print_results(void);
static void wifi_scan_start_async(void);

#endif