ifneq (,$(filter net_if,$(USEMODULE)))
	INCLUDES += -I$(RIOTBASE)/sys/net/include/
endif

ifneq (,$(filter nomac,$(USEMODULE)))
	INCLUDES += -I$(RIOTBASE)/sys/net/include/
endif

ifneq (,$(filter dev_eth_autoinit,$(USEMODULE)))
DIRS += $(RIOTBASE)/sys/auto_init/dev_eth
endif

include $(RIOTBASE)/Makefile.base
