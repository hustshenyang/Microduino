deps_config := \
	/c/esp32_idf/esp-idf/components/aws_iot/Kconfig \
	/c/esp32_idf/esp-idf/components/bt/Kconfig \
	/c/esp32_idf/esp-idf/components/esp32/Kconfig \
	/c/esp32_idf/esp-idf/components/ethernet/Kconfig \
	/c/esp32_idf/esp-idf/components/fatfs/Kconfig \
	/c/esp32_idf/esp-idf/components/freertos/Kconfig \
	/c/esp32_idf/esp-idf/components/log/Kconfig \
	/c/esp32_idf/esp-idf/components/lwip/Kconfig \
	/c/esp32_idf/esp-idf/components/mbedtls/Kconfig \
	/c/esp32_idf/esp-idf/components/openssl/Kconfig \
	/c/esp32_idf/esp-idf/components/spi_flash/Kconfig \
	/c/esp32_idf/esp32_quadCopter/main/Kconfig \
	/c/esp32_idf/esp-idf/components/bootloader/Kconfig.projbuild \
	/c/esp32_idf/esp-idf/components/esptool_py/Kconfig.projbuild \
	/c/esp32_idf/esp-idf/components/partition_table/Kconfig.projbuild \
	/c/esp32_idf/esp-idf/Kconfig

include/config/auto.conf: \
	$(deps_config)


$(deps_config): ;
