SRC = $(wildcard *.cc)
OBJ_DIR = obj/
OBJ = $(addprefix $(OBJ_DIR), $(SRC:.cc=.o))

# Libraries to resolve with pkgconfig
PKG_CFG_PATH=/usr/lib:/usr/local/lib
PKG_CFG_LIBS=libavcodec libavutil libswscale x11 vdpau \
	libva libva-drm libva-x11 opencv

CC				=g++
CC_FLAGS	=-std=c++0x -g
OUT_DIR		=bin

LIB_DIRS	=-L$(ARSDK_ROOT)/out/arsdk-native/staging/usr/lib -L"/usr/local/lib"
LIBS			=-larsal -larnetwork -larcommands -larcontroller -larstream -larstream2 -lardiscovery -larmedia -larnetworkal -larutils -lmux -lpomp

PKG_CC_FLAGS	:=$(shell PKG_CONFIG_PATH=$(PKG_CFG_PATH) pkg-config $(PKG_CFG_LIBS) --cflags)
PKG_LIBS			:=$(shell PKG_CONFIG_PATH=$(PKG_CFG_PATH) pkg-config $(PKG_CFG_LIBS) --libs)

CC_FLAGS	:=$(CC_FLAGS) $(PKG_CC_FLAGS)
LIBS			:=$(LIB_DIRS) $(LIBS) $(PKG_LIBS)

OUT=$(OUT_DIR)/BebopStream.out

all: setup $(OUT)

setup:
	@mkdir -p $(OUT_DIR)
	@mkdir -p $(OBJ_DIR)

$(OUT): check_env $(OBJ)
	@echo "> Linking..."
	@$(CC) -o $@ $(OBJ) $(LIBS) $(CC_FLAGS)

$(OBJ_DIR)%.o: %.cc
	@echo "> $<"
	@$(CC) -o $@ -I$(ARSDK_ROOT)/out/arsdk-native/staging/usr/include $< -c $(CC_FLAGS)

rungdb : $(OUT)
	@env LD_LIBRARY_PATH=$(ARSDK_ROOT)/out/arsdk-native/staging/usr/lib gdb ./$(OUT)

run : $(OUT)
	@env LD_LIBRARY_PATH=$(ARSDK_ROOT)/out/arsdk-native/staging/usr/lib ./$(OUT)

debug : $(OUT)
	@env LD_LIBRARY_PATH=$(ARSDK_ROOT)/out/arsdk-native/staging/usr/lib gdb ./$(OUT)

check_env:
ifndef ARSDK_ROOT
	$(error ARSDK_ROOT not defined. Please define it to the root folder of the SDK before calling this makefile)
endif

clean:
	@rm -f $(OUT) $(OBJ)

