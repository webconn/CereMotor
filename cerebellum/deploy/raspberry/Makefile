deploy_help+="\nraspberry - Flash robot MCU via network using Raspberry"

raspberry:
	scp $(IMAGENAME).bin $(CONFIG_DEPLOY_RASPBERRY_USER)@$(CONFIG_DEPLOY_RASPBERRY_IP):$(CONFIG_DEPLOY_RASPBERRY_PATH)
	ssh $(CONFIG_DEPLOY_RASPBERRY_USER)@$(CONFIG_DEPLOY_RASPBERRY_IP) deploy-stm32
