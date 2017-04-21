INC_PATH=inc
SRC_PATH=src
BUILD_PATH=build
LIBSCCC_INC_PATH?=lib/sccc/inc
LIBSCCC_BIN_PATH?=lib/sccc/lib
LIBSCCC_BIN?=sccc
OUT_EXE=inno14
OUT_EXE_SUFFIX=.elf
OUT_EXE_PATH=bin
OUT_OBJ_PATH=obj

TOOLCHAIN_PREFIX=arm-none-eabi-
CC=$(TOOLCHAIN_PREFIX)gcc
CXX=$(TOOLCHAIN_PREFIX)g++
AR=$(TOOLCHAIN_PREFIX)ar

# Additional include dirs and libs. libsccc will be added automatically
ALL_INC_PATHS=$(INC_PATH) $(SRC_PATH)
ALL_LIB_PATHS=
ALL_LIBS=

# Additional symbols
ALL_SYMBOLS=

# Basically you are not going to mess up with things below

ifeq ($(OS),Windows_NT)
WIN32=1
$(info Host = Win32)

else
UNIX=1
$(info Host = *nix)

endif

ifneq ($(firstword $(sort $(MAKE_VERSION) 3.81)),3.81)
$(error Require GNU Make 3.81 or newer)

else
$(info Make version = $(MAKE_VERSION))

endif

$(info User include paths = $(ALL_INC_PATHS))
$(info User lib paths = $(ALL_LIB_PATHS))
$(info User libs = $(ALL_LIBS))
$(info User symbols = $(ALL_SYMBOLS))

.DEFAULT_GOAL:=all

CCFLAGS=
CPPFLAGS=
ARFLAGS=
BIN_SUFFIX=
LDFLAGS=
LDLIBS=

CPPFLAGS+=$(addprefix -I,$(ALL_INC_PATHS)) -I$(LIBSCCC_INC_PATH)
CPPFLAGS+=$(addprefix -D,$(ALL_SYMBOLS))
CPPFLAGS+=-MMD

CCFLAGS+=-fmessage-length=0
CCFLAGS+=-flto -ffunction-sections -fdata-sections
CCFLAGS+=-fno-strict-aliasing
CCFLAGS+=-Wall -Wextra

ifeq ($(SCCC_BUILD),DEBUG)
BIN_SUFFIX:=$(BIN_SUFFIX)-d
CPPFLAGS+=-DDEBUG=1
CCFLAGS+=-O0 -g3
$(info Build = DEBUG)

else ifeq ($(SCCC_BUILD),RELEASE)
BIN_SUFFIX:=$(BIN_SUFFIX)-r
CPPFLAGS+=-DRELEASE=1 -DNDEBUG
CCFLAGS+=-O2 -g0
$(info Build = RELEASE)

else
$(warning Unknown build type, defaulting to DEBUG (set SCCC_BUILD))
BIN_SUFFIX:=$(BIN_SUFFIX)-d
CPPFLAGS+=-DDEBUG=1
CCFLAGS+=-O0 -g3
$(info Build = DEBUG)

endif

include MakeConfig.inc

ifeq ($(SCCC_MCU),MK60DZ10)
CPPFLAGS+=-DMK60DZ10=1
CCFLAGS+=-mthumb -mthumb-interwork -mcpu=cortex-m4
CCFLAGS+=-msoft-float -mfloat-abi=soft
LDFLAGS+=-mthumb -mthumb-interwork -mcpu=cortex-m4
LDFLAGS+=-msoft-float -mfloat-abi=soft
LDFLAGS+=-T $(BUILD_PATH)/d10.ld
$(info MCU sub-family = MK60DZ10)

else ifeq ($(SCCC_MCU),MK60D10)
CPPFLAGS+=-DMK60D10=1
CCFLAGS+=-mthumb -mthumb-interwork -mcpu=cortex-m4
CCFLAGS+=-msoft-float -mfloat-abi=soft
LDFLAGS+=-mthumb -mthumb-interwork -mcpu=cortex-m4
LDFLAGS+=-msoft-float -mfloat-abi=soft
LDFLAGS+=-T $(BUILD_PATH)/d10.ld
$(info MCU sub-family = MK60D10)

else ifeq ($(SCCC_MCU),MK60F15)
CPPFLAGS+=-DMK60F15=1
CCFLAGS+=-mthumb -mthumb-interwork -mcpu=cortex-m4
CCFLAGS+=-mfpu=fpv4-sp-d16 -mfloat-abi=hard
LDFLAGS+=-mthumb -mthumb-interwork -mcpu=cortex-m4
LDFLAGS+=-mfpu=fpv4-sp-d16 -mfloat-abi=hard
LDFLAGS+=-T $(BUILD_PATH)/f15.ld
$(info MCU sub-family = MK60F15)

else
$(error Missing/Unknown MCU identifier '$(SCCC_MCU)' (set SCCC_MCU))

endif

ifeq ($(MAKECMDGOALS),dry)
CCFLAGS+=-fsyntax-only
$(info Performing dry run (no binary))
endif

# End of common CCFLAGS
CXXFLAGS:=$(CCFLAGS)

CCFLAGS+=-std=gnu99

CXXFLAGS+=-std=gnu++11
CXXFLAGS+=-pedantic
CXXFLAGS+=-fno-exceptions -fno-rtti

ARFLAGS+=-r

# LDFLAGS+=-nostartfiles
LDFLAGS+=-specs=nano.specs -u _printf_float
LDFLAGS+=-Wl,--gc-sections
LDFLAGS+=-Wl,-Map=$(OUT_EXE_PATH)/$(OUT_EXE)$(BIN_SUFFIX).map

LDFLAGS+=$(addprefix -L,$(ALL_LIB_PATHS)) -L$(LIBSCCC_BIN_PATH)
LDLIBS+=$(addprefix -l,$(ALL_LIBS)) -l$(LIBSCCC_BIN)$(BIN_SUFFIX)

# End setting flags

$(info Building $(OUT_EXE)$(BIN_SUFFIX)$(OUT_EXE_SUFFIX))

ifdef WIN32
rwildcard=$(wildcard $1/$2) $(foreach d,$(wildcard $1/*),$(call rwildcard,$(d),$2))
SRC_FILES:=$(call rwildcard,$(SRC_PATH),*.c)
SRC_FILES:=$(SRC_FILES) $(call rwildcard,$(SRC_PATH),*.S)
SRC_FILES:=$(SRC_FILES) $(call rwildcard,$(SRC_PATH),*.cpp)

else ifdef UNIX
SRC_FILES:=$(shell find $(SRC_PATH) -type f -name *.c -o -name *.S -o -name *.cpp)

endif

OBJ_FILES:=$(SRC_FILES:$(SRC_PATH)/%.c=$(OUT_OBJ_PATH)/%.o)
OBJ_FILES:=$(OBJ_FILES:$(SRC_PATH)/%.S=$(OUT_OBJ_PATH)/%.o)
OBJ_FILES:=$(OBJ_FILES:$(SRC_PATH)/%.cpp=$(OUT_OBJ_PATH)/%.o)
OBJ_FILES:=$(OBJ_FILES:%.o=%$(BIN_SUFFIX).o)

DEPENDS:=$(OBJ_FILES:.o=.d)
-include $(DEPENDS)

OUT_DIRS:=$(sort $(dir $(OBJ_FILES)))
ifdef WIN32
$(shell mkdir $(subst /,\,$(OUT_DIRS)) bin > nul)

else ifdef UNIX
$(shell mkdir -p $(OUT_DIRS) bin)

endif

.PHONY: all clean dry

all: $(OUT_EXE_PATH)/$(OUT_EXE)$(BIN_SUFFIX)$(OUT_EXE_SUFFIX)

dry: $(OBJ_FILES)

.SECONDEXPANSION:

$(OUT_EXE_PATH)/$(OUT_EXE)$(BIN_SUFFIX)$(OUT_EXE_SUFFIX): $(OBJ_FILES) $(LIBSCCC_BIN_PATH)/lib$(LIBSCCC_BIN)$(BIN_SUFFIX).a
	$(info Linking objects)
	@$(CXX) $(LDFLAGS) -o $@ $(OBJ_FILES) $(LDLIBS)

$(OUT_OBJ_PATH)/%.o: $$(subst $(BIN_SUFFIX),,$(SRC_PATH)/%.c)
	$(info Compiling $(<))
	@$(CC) $(CPPFLAGS) $(CCFLAGS) -c -o $@ $<

$(OUT_OBJ_PATH)/%.o: $$(subst $(BIN_SUFFIX),,$(SRC_PATH)/%.S)
	$(info Compiling $(<))
	@$(CC) $(CPPFLAGS) $(CCFLAGS) -c -o $@ $<

$(OUT_OBJ_PATH)/%.o: $$(subst $(BIN_SUFFIX),,$(SRC_PATH)/%.cpp)
	$(info Compiling $(<))
	@$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

clean:
	$(info Cleaning $(<))
ifdef WIN32
	@rmdir /s /q $(OUT_OBJ_PATH) $(OUT_LIB_PATH)

else ifdef UNIX
	@rm -f $(OUT_LIB_PATH)/*.a
	@find $(OUT_OBJ_PATH) -type f \( -name *.o -o -name *.d \) -exec rm -f {} \;

endif
