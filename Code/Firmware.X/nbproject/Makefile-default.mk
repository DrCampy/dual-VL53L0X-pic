#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Firmware.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Firmware.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS
SUB_IMAGE_ADDRESS_COMMAND=--image-address $(SUB_IMAGE_ADDRESS)
else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=src/SlaveI2C.c src/configuration_bits.c src/data_storage.c src/interrupts.c src/main.c src/sensor.c src/system.c src/traps.c "src/DEEE/DEE Emulation 16-bit/DEE Emulation 16-bit.c" src/Api/src/core/vl53l0x_api.c src/Api/src/core/vl53l0x_api_calibration.c src/Api/src/core/vl53l0x_api_core.c src/Api/src/core/vl53l0x_api_ranging.c src/Api/src/core/vl53l0x_api_strings.c src/Api/src/platform/vl53l0x_i2c_PIC24FJ64GA702.c src/Api/src/platform/vl53l0x_platform.c "src/DEEE/DEE Emulation 16-bit/FlashOperations.s"

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/src/SlaveI2C.o ${OBJECTDIR}/src/configuration_bits.o ${OBJECTDIR}/src/data_storage.o ${OBJECTDIR}/src/interrupts.o ${OBJECTDIR}/src/main.o ${OBJECTDIR}/src/sensor.o ${OBJECTDIR}/src/system.o ${OBJECTDIR}/src/traps.o "${OBJECTDIR}/src/DEEE/DEE Emulation 16-bit/DEE Emulation 16-bit.o" ${OBJECTDIR}/src/Api/src/core/vl53l0x_api.o ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_calibration.o ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_core.o ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_ranging.o ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_strings.o ${OBJECTDIR}/src/Api/src/platform/vl53l0x_i2c_PIC24FJ64GA702.o ${OBJECTDIR}/src/Api/src/platform/vl53l0x_platform.o "${OBJECTDIR}/src/DEEE/DEE Emulation 16-bit/FlashOperations.o"
POSSIBLE_DEPFILES=${OBJECTDIR}/src/SlaveI2C.o.d ${OBJECTDIR}/src/configuration_bits.o.d ${OBJECTDIR}/src/data_storage.o.d ${OBJECTDIR}/src/interrupts.o.d ${OBJECTDIR}/src/main.o.d ${OBJECTDIR}/src/sensor.o.d ${OBJECTDIR}/src/system.o.d ${OBJECTDIR}/src/traps.o.d "${OBJECTDIR}/src/DEEE/DEE Emulation 16-bit/DEE Emulation 16-bit.o.d" ${OBJECTDIR}/src/Api/src/core/vl53l0x_api.o.d ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_calibration.o.d ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_core.o.d ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_ranging.o.d ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_strings.o.d ${OBJECTDIR}/src/Api/src/platform/vl53l0x_i2c_PIC24FJ64GA702.o.d ${OBJECTDIR}/src/Api/src/platform/vl53l0x_platform.o.d "${OBJECTDIR}/src/DEEE/DEE Emulation 16-bit/FlashOperations.o.d"

# Object Files
OBJECTFILES=${OBJECTDIR}/src/SlaveI2C.o ${OBJECTDIR}/src/configuration_bits.o ${OBJECTDIR}/src/data_storage.o ${OBJECTDIR}/src/interrupts.o ${OBJECTDIR}/src/main.o ${OBJECTDIR}/src/sensor.o ${OBJECTDIR}/src/system.o ${OBJECTDIR}/src/traps.o ${OBJECTDIR}/src/DEEE/DEE\ Emulation\ 16-bit/DEE\ Emulation\ 16-bit.o ${OBJECTDIR}/src/Api/src/core/vl53l0x_api.o ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_calibration.o ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_core.o ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_ranging.o ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_strings.o ${OBJECTDIR}/src/Api/src/platform/vl53l0x_i2c_PIC24FJ64GA702.o ${OBJECTDIR}/src/Api/src/platform/vl53l0x_platform.o ${OBJECTDIR}/src/DEEE/DEE\ Emulation\ 16-bit/FlashOperations.o

# Source Files
SOURCEFILES=src/SlaveI2C.c src/configuration_bits.c src/data_storage.c src/interrupts.c src/main.c src/sensor.c src/system.c src/traps.c src/DEEE/DEE Emulation 16-bit/DEE Emulation 16-bit.c src/Api/src/core/vl53l0x_api.c src/Api/src/core/vl53l0x_api_calibration.c src/Api/src/core/vl53l0x_api_core.c src/Api/src/core/vl53l0x_api_ranging.c src/Api/src/core/vl53l0x_api_strings.c src/Api/src/platform/vl53l0x_i2c_PIC24FJ64GA702.c src/Api/src/platform/vl53l0x_platform.c src/DEEE/DEE Emulation 16-bit/FlashOperations.s



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/Firmware.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=24FJ256GA702
MP_LINKER_FILE_OPTION=,--script=p24FJ256GA702.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/src/SlaveI2C.o: src/SlaveI2C.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src" 
	@${RM} ${OBJECTDIR}/src/SlaveI2C.o.d 
	@${RM} ${OBJECTDIR}/src/SlaveI2C.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/SlaveI2C.c  -o ${OBJECTDIR}/src/SlaveI2C.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/SlaveI2C.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/SlaveI2C.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/configuration_bits.o: src/configuration_bits.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src" 
	@${RM} ${OBJECTDIR}/src/configuration_bits.o.d 
	@${RM} ${OBJECTDIR}/src/configuration_bits.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/configuration_bits.c  -o ${OBJECTDIR}/src/configuration_bits.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/configuration_bits.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/configuration_bits.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/data_storage.o: src/data_storage.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src" 
	@${RM} ${OBJECTDIR}/src/data_storage.o.d 
	@${RM} ${OBJECTDIR}/src/data_storage.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/data_storage.c  -o ${OBJECTDIR}/src/data_storage.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/data_storage.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/data_storage.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/interrupts.o: src/interrupts.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src" 
	@${RM} ${OBJECTDIR}/src/interrupts.o.d 
	@${RM} ${OBJECTDIR}/src/interrupts.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/interrupts.c  -o ${OBJECTDIR}/src/interrupts.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/interrupts.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/interrupts.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/main.o: src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src" 
	@${RM} ${OBJECTDIR}/src/main.o.d 
	@${RM} ${OBJECTDIR}/src/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/main.c  -o ${OBJECTDIR}/src/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/main.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/sensor.o: src/sensor.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src" 
	@${RM} ${OBJECTDIR}/src/sensor.o.d 
	@${RM} ${OBJECTDIR}/src/sensor.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/sensor.c  -o ${OBJECTDIR}/src/sensor.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/sensor.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/sensor.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/system.o: src/system.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src" 
	@${RM} ${OBJECTDIR}/src/system.o.d 
	@${RM} ${OBJECTDIR}/src/system.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/system.c  -o ${OBJECTDIR}/src/system.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/system.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/system.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/traps.o: src/traps.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src" 
	@${RM} ${OBJECTDIR}/src/traps.o.d 
	@${RM} ${OBJECTDIR}/src/traps.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/traps.c  -o ${OBJECTDIR}/src/traps.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/traps.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/traps.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/DEEE/DEE\ Emulation\ 16-bit/DEE\ Emulation\ 16-bit.o: src/DEEE/DEE\ Emulation\ 16-bit/DEE\ Emulation\ 16-bit.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src/DEEE/DEE Emulation 16-bit" 
	@${RM} "${OBJECTDIR}/src/DEEE/DEE Emulation 16-bit/DEE Emulation 16-bit.o".d 
	@${RM} "${OBJECTDIR}/src/DEEE/DEE Emulation 16-bit/DEE Emulation 16-bit.o" 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "src/DEEE/DEE Emulation 16-bit/DEE Emulation 16-bit.c"  -o "${OBJECTDIR}/src/DEEE/DEE Emulation 16-bit/DEE Emulation 16-bit.o"  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/DEEE/DEE Emulation 16-bit/DEE Emulation 16-bit.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/DEEE/DEE Emulation 16-bit/DEE Emulation 16-bit.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/Api/src/core/vl53l0x_api.o: src/Api/src/core/vl53l0x_api.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src/Api/src/core" 
	@${RM} ${OBJECTDIR}/src/Api/src/core/vl53l0x_api.o.d 
	@${RM} ${OBJECTDIR}/src/Api/src/core/vl53l0x_api.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/Api/src/core/vl53l0x_api.c  -o ${OBJECTDIR}/src/Api/src/core/vl53l0x_api.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/Api/src/core/vl53l0x_api.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/Api/src/core/vl53l0x_api.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/Api/src/core/vl53l0x_api_calibration.o: src/Api/src/core/vl53l0x_api_calibration.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src/Api/src/core" 
	@${RM} ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_calibration.o.d 
	@${RM} ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_calibration.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/Api/src/core/vl53l0x_api_calibration.c  -o ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_calibration.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/Api/src/core/vl53l0x_api_calibration.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/Api/src/core/vl53l0x_api_calibration.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/Api/src/core/vl53l0x_api_core.o: src/Api/src/core/vl53l0x_api_core.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src/Api/src/core" 
	@${RM} ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_core.o.d 
	@${RM} ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_core.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/Api/src/core/vl53l0x_api_core.c  -o ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_core.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/Api/src/core/vl53l0x_api_core.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/Api/src/core/vl53l0x_api_core.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/Api/src/core/vl53l0x_api_ranging.o: src/Api/src/core/vl53l0x_api_ranging.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src/Api/src/core" 
	@${RM} ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_ranging.o.d 
	@${RM} ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_ranging.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/Api/src/core/vl53l0x_api_ranging.c  -o ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_ranging.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/Api/src/core/vl53l0x_api_ranging.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/Api/src/core/vl53l0x_api_ranging.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/Api/src/core/vl53l0x_api_strings.o: src/Api/src/core/vl53l0x_api_strings.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src/Api/src/core" 
	@${RM} ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_strings.o.d 
	@${RM} ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_strings.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/Api/src/core/vl53l0x_api_strings.c  -o ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_strings.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/Api/src/core/vl53l0x_api_strings.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/Api/src/core/vl53l0x_api_strings.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/Api/src/platform/vl53l0x_i2c_PIC24FJ64GA702.o: src/Api/src/platform/vl53l0x_i2c_PIC24FJ64GA702.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src/Api/src/platform" 
	@${RM} ${OBJECTDIR}/src/Api/src/platform/vl53l0x_i2c_PIC24FJ64GA702.o.d 
	@${RM} ${OBJECTDIR}/src/Api/src/platform/vl53l0x_i2c_PIC24FJ64GA702.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/Api/src/platform/vl53l0x_i2c_PIC24FJ64GA702.c  -o ${OBJECTDIR}/src/Api/src/platform/vl53l0x_i2c_PIC24FJ64GA702.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/Api/src/platform/vl53l0x_i2c_PIC24FJ64GA702.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/Api/src/platform/vl53l0x_i2c_PIC24FJ64GA702.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/Api/src/platform/vl53l0x_platform.o: src/Api/src/platform/vl53l0x_platform.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src/Api/src/platform" 
	@${RM} ${OBJECTDIR}/src/Api/src/platform/vl53l0x_platform.o.d 
	@${RM} ${OBJECTDIR}/src/Api/src/platform/vl53l0x_platform.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/Api/src/platform/vl53l0x_platform.c  -o ${OBJECTDIR}/src/Api/src/platform/vl53l0x_platform.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/Api/src/platform/vl53l0x_platform.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/Api/src/platform/vl53l0x_platform.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/src/SlaveI2C.o: src/SlaveI2C.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src" 
	@${RM} ${OBJECTDIR}/src/SlaveI2C.o.d 
	@${RM} ${OBJECTDIR}/src/SlaveI2C.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/SlaveI2C.c  -o ${OBJECTDIR}/src/SlaveI2C.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/SlaveI2C.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/SlaveI2C.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/configuration_bits.o: src/configuration_bits.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src" 
	@${RM} ${OBJECTDIR}/src/configuration_bits.o.d 
	@${RM} ${OBJECTDIR}/src/configuration_bits.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/configuration_bits.c  -o ${OBJECTDIR}/src/configuration_bits.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/configuration_bits.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/configuration_bits.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/data_storage.o: src/data_storage.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src" 
	@${RM} ${OBJECTDIR}/src/data_storage.o.d 
	@${RM} ${OBJECTDIR}/src/data_storage.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/data_storage.c  -o ${OBJECTDIR}/src/data_storage.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/data_storage.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/data_storage.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/interrupts.o: src/interrupts.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src" 
	@${RM} ${OBJECTDIR}/src/interrupts.o.d 
	@${RM} ${OBJECTDIR}/src/interrupts.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/interrupts.c  -o ${OBJECTDIR}/src/interrupts.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/interrupts.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/interrupts.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/main.o: src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src" 
	@${RM} ${OBJECTDIR}/src/main.o.d 
	@${RM} ${OBJECTDIR}/src/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/main.c  -o ${OBJECTDIR}/src/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/main.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/sensor.o: src/sensor.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src" 
	@${RM} ${OBJECTDIR}/src/sensor.o.d 
	@${RM} ${OBJECTDIR}/src/sensor.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/sensor.c  -o ${OBJECTDIR}/src/sensor.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/sensor.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/sensor.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/system.o: src/system.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src" 
	@${RM} ${OBJECTDIR}/src/system.o.d 
	@${RM} ${OBJECTDIR}/src/system.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/system.c  -o ${OBJECTDIR}/src/system.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/system.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/system.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/traps.o: src/traps.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src" 
	@${RM} ${OBJECTDIR}/src/traps.o.d 
	@${RM} ${OBJECTDIR}/src/traps.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/traps.c  -o ${OBJECTDIR}/src/traps.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/traps.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/traps.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/DEEE/DEE\ Emulation\ 16-bit/DEE\ Emulation\ 16-bit.o: src/DEEE/DEE\ Emulation\ 16-bit/DEE\ Emulation\ 16-bit.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src/DEEE/DEE Emulation 16-bit" 
	@${RM} "${OBJECTDIR}/src/DEEE/DEE Emulation 16-bit/DEE Emulation 16-bit.o".d 
	@${RM} "${OBJECTDIR}/src/DEEE/DEE Emulation 16-bit/DEE Emulation 16-bit.o" 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "src/DEEE/DEE Emulation 16-bit/DEE Emulation 16-bit.c"  -o "${OBJECTDIR}/src/DEEE/DEE Emulation 16-bit/DEE Emulation 16-bit.o"  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/DEEE/DEE Emulation 16-bit/DEE Emulation 16-bit.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/DEEE/DEE Emulation 16-bit/DEE Emulation 16-bit.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/Api/src/core/vl53l0x_api.o: src/Api/src/core/vl53l0x_api.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src/Api/src/core" 
	@${RM} ${OBJECTDIR}/src/Api/src/core/vl53l0x_api.o.d 
	@${RM} ${OBJECTDIR}/src/Api/src/core/vl53l0x_api.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/Api/src/core/vl53l0x_api.c  -o ${OBJECTDIR}/src/Api/src/core/vl53l0x_api.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/Api/src/core/vl53l0x_api.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/Api/src/core/vl53l0x_api.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/Api/src/core/vl53l0x_api_calibration.o: src/Api/src/core/vl53l0x_api_calibration.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src/Api/src/core" 
	@${RM} ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_calibration.o.d 
	@${RM} ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_calibration.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/Api/src/core/vl53l0x_api_calibration.c  -o ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_calibration.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/Api/src/core/vl53l0x_api_calibration.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/Api/src/core/vl53l0x_api_calibration.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/Api/src/core/vl53l0x_api_core.o: src/Api/src/core/vl53l0x_api_core.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src/Api/src/core" 
	@${RM} ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_core.o.d 
	@${RM} ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_core.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/Api/src/core/vl53l0x_api_core.c  -o ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_core.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/Api/src/core/vl53l0x_api_core.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/Api/src/core/vl53l0x_api_core.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/Api/src/core/vl53l0x_api_ranging.o: src/Api/src/core/vl53l0x_api_ranging.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src/Api/src/core" 
	@${RM} ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_ranging.o.d 
	@${RM} ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_ranging.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/Api/src/core/vl53l0x_api_ranging.c  -o ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_ranging.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/Api/src/core/vl53l0x_api_ranging.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/Api/src/core/vl53l0x_api_ranging.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/Api/src/core/vl53l0x_api_strings.o: src/Api/src/core/vl53l0x_api_strings.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src/Api/src/core" 
	@${RM} ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_strings.o.d 
	@${RM} ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_strings.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/Api/src/core/vl53l0x_api_strings.c  -o ${OBJECTDIR}/src/Api/src/core/vl53l0x_api_strings.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/Api/src/core/vl53l0x_api_strings.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/Api/src/core/vl53l0x_api_strings.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/Api/src/platform/vl53l0x_i2c_PIC24FJ64GA702.o: src/Api/src/platform/vl53l0x_i2c_PIC24FJ64GA702.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src/Api/src/platform" 
	@${RM} ${OBJECTDIR}/src/Api/src/platform/vl53l0x_i2c_PIC24FJ64GA702.o.d 
	@${RM} ${OBJECTDIR}/src/Api/src/platform/vl53l0x_i2c_PIC24FJ64GA702.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/Api/src/platform/vl53l0x_i2c_PIC24FJ64GA702.c  -o ${OBJECTDIR}/src/Api/src/platform/vl53l0x_i2c_PIC24FJ64GA702.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/Api/src/platform/vl53l0x_i2c_PIC24FJ64GA702.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/Api/src/platform/vl53l0x_i2c_PIC24FJ64GA702.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/Api/src/platform/vl53l0x_platform.o: src/Api/src/platform/vl53l0x_platform.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src/Api/src/platform" 
	@${RM} ${OBJECTDIR}/src/Api/src/platform/vl53l0x_platform.o.d 
	@${RM} ${OBJECTDIR}/src/Api/src/platform/vl53l0x_platform.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/Api/src/platform/vl53l0x_platform.c  -o ${OBJECTDIR}/src/Api/src/platform/vl53l0x_platform.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/Api/src/platform/vl53l0x_platform.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -msmall-data -O0 -DUSE_I2C_2V8 -msmart-io=1 -Wall -msfr-warn=off    -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/Api/src/platform/vl53l0x_platform.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/src/DEEE/DEE\ Emulation\ 16-bit/FlashOperations.o: src/DEEE/DEE\ Emulation\ 16-bit/FlashOperations.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src/DEEE/DEE Emulation 16-bit" 
	@${RM} "${OBJECTDIR}/src/DEEE/DEE Emulation 16-bit/FlashOperations.o".d 
	@${RM} "${OBJECTDIR}/src/DEEE/DEE Emulation 16-bit/FlashOperations.o" 
	${MP_CC} $(MP_EXTRA_AS_PRE)  "src/DEEE/DEE Emulation 16-bit/FlashOperations.s"  -o "${OBJECTDIR}/src/DEEE/DEE Emulation 16-bit/FlashOperations.o"  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG   -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -Wa,-MD,"${OBJECTDIR}/src/DEEE/DEE Emulation 16-bit/FlashOperations.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,,-g,--no-relax$(MP_EXTRA_AS_POST)  -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/DEEE/DEE Emulation 16-bit/FlashOperations.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
else
${OBJECTDIR}/src/DEEE/DEE\ Emulation\ 16-bit/FlashOperations.o: src/DEEE/DEE\ Emulation\ 16-bit/FlashOperations.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/src/DEEE/DEE Emulation 16-bit" 
	@${RM} "${OBJECTDIR}/src/DEEE/DEE Emulation 16-bit/FlashOperations.o".d 
	@${RM} "${OBJECTDIR}/src/DEEE/DEE Emulation 16-bit/FlashOperations.o" 
	${MP_CC} $(MP_EXTRA_AS_PRE)  "src/DEEE/DEE Emulation 16-bit/FlashOperations.s"  -o "${OBJECTDIR}/src/DEEE/DEE Emulation 16-bit/FlashOperations.o"  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -Wa,-MD,"${OBJECTDIR}/src/DEEE/DEE Emulation 16-bit/FlashOperations.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax$(MP_EXTRA_AS_POST)  -mdfp=${DFP_DIR}/xc16
	@${FIXDEPS} "${OBJECTDIR}/src/DEEE/DEE Emulation 16-bit/FlashOperations.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/Firmware.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Firmware.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG=__DEBUG   -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core"     -Wl,--local-stack,,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D__DEBUG=__DEBUG,,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST)  -mdfp=${DFP_DIR}/xc16 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/Firmware.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Firmware.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"src/DEEE/Include/DEE Emulation 16-bit" -I"src/Api/inc/platform" -I"src/Api/inc/core" -Wl,--local-stack,,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST)  -mdfp=${DFP_DIR}/xc16 
	${MP_CC_DIR}\\xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/Firmware.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf   -mdfp=${DFP_DIR}/xc16 
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
