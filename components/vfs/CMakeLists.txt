idf_component_register(SRCS "vfs.c"
                            "vfs_eventfd.c"
                            "vfs_uart.c"
                            "vfs_semihost.c"
                    INCLUDE_DIRS include
                    PRIV_REQUIRES driver)

if(CONFIG_ESP_CONSOLE_USB_CDC)
    target_sources(${COMPONENT_LIB} PRIVATE "vfs_cdcacm.c")
endif()

if(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    target_sources(${COMPONENT_LIB} PRIVATE "vfs_usb_serial_jtag.c")
endif()

# Some newlib syscalls are implemented in vfs.c, make sure these are always
# seen by the linker
target_link_libraries(${COMPONENT_LIB} INTERFACE "-u vfs_include_syscalls_impl")
