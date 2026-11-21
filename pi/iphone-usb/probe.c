// Read-only libusb access check; does not change USB configuration.
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <libusb.h>
int main(void) {
    if (geteuid() != 0) {
        fprintf(stderr, "Run with sudo: the installed QNX libusb backend crashes without USB access privileges.\n");
        return 1;
    }
    libusb_context *ctx = NULL;
    int result = libusb_init(&ctx);
    if (result < 0) { fprintf(stderr, "init: %s\n", libusb_error_name(result)); return 1; }
    libusb_device **devices = NULL;
    ssize_t count = libusb_get_device_list(ctx, &devices);
    int found = 0;
    if (count < 0) { fprintf(stderr, "list: %s\n", libusb_error_name((int)count)); libusb_exit(ctx); return 1; }
    for (ssize_t i = 0; i < count; i++) {
        struct libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(devices[i], &desc) < 0 || desc.idVendor != 0x05ac) continue;
        found = 1;
        printf("Apple USB %04x:%04x bus=%u address=%u\n", desc.idVendor, desc.idProduct,
               libusb_get_bus_number(devices[i]), libusb_get_device_address(devices[i]));
        libusb_device_handle *handle = NULL;
        result = libusb_open(devices[i], &handle);
        printf("open: %s\n", libusb_error_name(result));
        if (result == 0) {
            int configuration = 0;
            result = libusb_get_configuration(handle, &configuration);
            printf("configuration: %d (%s)\n", configuration, libusb_error_name(result));
            libusb_close(handle);
        }
    }
    libusb_free_device_list(devices, 1);
    libusb_exit(ctx);
    return found ? 0 : 2;
}
