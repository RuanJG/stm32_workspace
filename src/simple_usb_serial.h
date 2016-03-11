int simple_usb_serial_write(uint8_t *buff,int len);
int simple_usb_serial_write_block(uint8_t *buff,int len);
int simple_usb_serial_read(uint8_t *buff,int maxlen);
void simple_usb_serial_event();
void simple_usb_serial_init();
extern unsigned int write_wait_count ;
//may be no used
inline int simple_usb_serial_check_connected();
inline int simple_usb_serial_check_available();
inline int simple_usb_serial_check_free_space();
int simple_usb_serial_write_push();
