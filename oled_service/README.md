# Module: oled_service_if
## Description: the interface support to receive oled data from oled_service and send oled data to system.

## 1. Turning on and off OLED device

To send warning message, using the function:

    on_off_screen(uint32_t status);

<ins>**Note:**</ins>

**status:** *value indicate user requirement to turn on or off device*

<ins>**Example:**</ins>

**Step 1 :** Creating instance <code>oled_service_if</code>.

    oled_service_if *oled = new oled_service_if("banvien.test");

**Step 2:** Sending status of OLED user want to control to dbus.

    uint32_t status = off;
    oled->on_off_screen(status);

## 2. Sending battery percent to OLED

To send battery percent value, using the function:
    
    display_battery_percent(uint32_t percent);

<ins>**Note:**</ins>

**percent:** *battery percentage value*

**Attention:** *Please note that battery percent value not larger than 100*

<ins>**Example:**</ins>

**Step 1 :** Creating instance <code>oled_service_if</code>.

    oled_service_if *oled = new oled_service_if("banvien.test");

**Step 2:** Sending battery percent value to dbus.

    uint32_t percent = 50;
    oled->display_battery_percent(percent);

## 3. Updating wifi access point status to OLED

To send warning message, using the function:
    
    on_off_access_point(uint32_t status);

<ins>**Note:**</ins>

**status:** *value indicate wifi access point status*

<ins>**Example:**</ins>

**Step 1 :** Creating instance <code>oled_service_if</code>.

    oled_service_if *oled = new oled_service_if("banvien.test");

**Step 2:** Sending wifi access point status value to dbus.

    uint32_t status = on;
    oled->display_warning_message(mes);

## 4. Sending number of wifi client to OLED

To send number of wifi client, using the function:
    
    display_number_wifi_client(uint32_t number_client);

<ins>**Note:**</ins>

**number_client:** *number of client connect to wifi*
**Attention:** *Please note that number of clients cannot higher than 9*

<ins>**Example:**</ins>

**Step 1 :** Creating instance <code>oled_service_if</code>.

    oled_service_if *oled = new oled_service_if("banvien.test");

**Step 2:** Sending number of wifi client to dbus.

    uint32_t number_client = 7;
    oled->display_number_wifi_client(number_client);

## 5. Sending warning message to OLED

To send warning message, using the function:
    
    display_warning_message(string mes);

<ins>**Note:**</ins>

**mes:** *warning message that is sent to OLED*

<ins>**Example:**</ins>

**Step 1 :** Creating instance <code>oled_service_if</code>.

    oled_service_if *oled = new oled_service_if("banvien.test");

**Step 2:** Sending battery percent value to dbus.

    string mes = "Error...";
    oled->display_warning_message(mes);
