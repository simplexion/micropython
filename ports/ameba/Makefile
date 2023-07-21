AMBD_SDK=$(shell pwd)/../../lib/ambd_sdk/
PROJECT_BASE=$(AMBD_SDK)/project/realtek_amebaD_va0_example/GCC-RELEASE/
IMAGE_DIR=$(shell pwd)/image/

all:
	rm -rf $(IMAGE_DIR)

	make -C lp AMBD_SDK=$(AMBD_SDK)
	make -C hp AMBD_SDK=$(AMBD_SDK)

	mkdir -p $(IMAGE_DIR)

	cp $(AMBD_SDK)/tools/AmebaD/Image_Tool/imgtool_flashloader_amebad.bin $(IMAGE_DIR)/
	cp $(PROJECT_BASE)/project_lp/asdk/image/km0_boot_all.bin $(IMAGE_DIR)/
	cp $(PROJECT_BASE)/project_hp/asdk/image/km4_boot_all.bin $(IMAGE_DIR)/
	cp $(PROJECT_BASE)/project_hp/asdk/image/km0_km4_image2.bin $(IMAGE_DIR)/

clean:
	rm -rf $(IMAGE_DIR)
	make -C lp clean
	make -C hp clean
