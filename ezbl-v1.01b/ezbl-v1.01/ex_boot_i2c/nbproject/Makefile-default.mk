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
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/ex_boot_i2c.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/ex_boot_i2c.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=hardware_initializers/dspic33ep512gm710_explorer_16.c hardware_initializers/pic24fj128ga010_explorer_16.c hardware_initializers/dspic33ep512mu810_explorer_16.c hardware_initializers/pic24fj256gb110_explorer_16.c hardware_initializers/pic24fj256ga110_explorer_16.c hardware_initializers/pic24fj64ga004_explorer_16.c hardware_initializers/dspic33ep256gp506_explorer_16.c hardware_initializers/dspic33fj256gp710a_explorer_16.c hardware_initializers/pic24fj1024gb610_explorer_16.c main.c now.c i2c_fifo.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o ${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o ${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o ${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o ${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o ${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o ${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o ${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o ${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o ${OBJECTDIR}/main.o ${OBJECTDIR}/now.o ${OBJECTDIR}/i2c_fifo.o
POSSIBLE_DEPFILES=${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o.d ${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o.d ${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o.d ${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o.d ${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o.d ${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o.d ${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o.d ${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o.d ${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o.d ${OBJECTDIR}/main.o.d ${OBJECTDIR}/now.o.d ${OBJECTDIR}/i2c_fifo.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o ${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o ${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o ${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o ${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o ${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o ${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o ${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o ${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o ${OBJECTDIR}/main.o ${OBJECTDIR}/now.o ${OBJECTDIR}/i2c_fifo.o

# Source Files
SOURCEFILES=hardware_initializers/dspic33ep512gm710_explorer_16.c hardware_initializers/pic24fj128ga010_explorer_16.c hardware_initializers/dspic33ep512mu810_explorer_16.c hardware_initializers/pic24fj256gb110_explorer_16.c hardware_initializers/pic24fj256ga110_explorer_16.c hardware_initializers/pic24fj64ga004_explorer_16.c hardware_initializers/dspic33ep256gp506_explorer_16.c hardware_initializers/dspic33fj256gp710a_explorer_16.c hardware_initializers/pic24fj1024gb610_explorer_16.c main.c now.c i2c_fifo.c


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
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/ex_boot_i2c.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=24FJ64GA004
MP_LINKER_FILE_OPTION=,--script=ezbl_integration/ezbl_build_standalone.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o: hardware_initializers/dspic33ep512gm710_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/dspic33ep512gm710_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1    -omf=elf -ffunction-sections -fdata-sections -mlarge-code -mlarge-data -mconst-in-data -O1 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o: hardware_initializers/pic24fj128ga010_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/pic24fj128ga010_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1    -omf=elf -ffunction-sections -fdata-sections -mlarge-code -mlarge-data -mconst-in-data -O1 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o: hardware_initializers/dspic33ep512mu810_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/dspic33ep512mu810_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1    -omf=elf -ffunction-sections -fdata-sections -mlarge-code -mlarge-data -mconst-in-data -O1 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o: hardware_initializers/pic24fj256gb110_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/pic24fj256gb110_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1    -omf=elf -ffunction-sections -fdata-sections -mlarge-code -mlarge-data -mconst-in-data -O1 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o: hardware_initializers/pic24fj256ga110_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/pic24fj256ga110_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1    -omf=elf -ffunction-sections -fdata-sections -mlarge-code -mlarge-data -mconst-in-data -O1 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o: hardware_initializers/pic24fj64ga004_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/pic24fj64ga004_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1    -omf=elf -ffunction-sections -fdata-sections -mlarge-code -mlarge-data -mconst-in-data -O1 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o: hardware_initializers/dspic33ep256gp506_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/dspic33ep256gp506_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1    -omf=elf -ffunction-sections -fdata-sections -mlarge-code -mlarge-data -mconst-in-data -O1 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o: hardware_initializers/dspic33fj256gp710a_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/dspic33fj256gp710a_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1    -omf=elf -ffunction-sections -fdata-sections -mlarge-code -mlarge-data -mconst-in-data -O1 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o: hardware_initializers/pic24fj1024gb610_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/pic24fj1024gb610_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1    -omf=elf -ffunction-sections -fdata-sections -mlarge-code -mlarge-data -mconst-in-data -O1 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main.c  -o ${OBJECTDIR}/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/main.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1    -omf=elf -ffunction-sections -fdata-sections -mlarge-code -mlarge-data -mconst-in-data -O1 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/now.o: now.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/now.o.d 
	@${RM} ${OBJECTDIR}/now.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  now.c  -o ${OBJECTDIR}/now.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/now.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1    -omf=elf -ffunction-sections -fdata-sections -mlarge-code -mlarge-data -mconst-in-data -O1 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/now.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/i2c_fifo.o: i2c_fifo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/i2c_fifo.o.d 
	@${RM} ${OBJECTDIR}/i2c_fifo.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  i2c_fifo.c  -o ${OBJECTDIR}/i2c_fifo.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/i2c_fifo.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1    -omf=elf -ffunction-sections -fdata-sections -mlarge-code -mlarge-data -mconst-in-data -O1 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/i2c_fifo.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o: hardware_initializers/dspic33ep512gm710_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/dspic33ep512gm710_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o.d"        -g -omf=elf -ffunction-sections -fdata-sections -mlarge-code -mlarge-data -mconst-in-data -O1 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o: hardware_initializers/pic24fj128ga010_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/pic24fj128ga010_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o.d"        -g -omf=elf -ffunction-sections -fdata-sections -mlarge-code -mlarge-data -mconst-in-data -O1 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o: hardware_initializers/dspic33ep512mu810_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/dspic33ep512mu810_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o.d"        -g -omf=elf -ffunction-sections -fdata-sections -mlarge-code -mlarge-data -mconst-in-data -O1 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o: hardware_initializers/pic24fj256gb110_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/pic24fj256gb110_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o.d"        -g -omf=elf -ffunction-sections -fdata-sections -mlarge-code -mlarge-data -mconst-in-data -O1 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o: hardware_initializers/pic24fj256ga110_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/pic24fj256ga110_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o.d"        -g -omf=elf -ffunction-sections -fdata-sections -mlarge-code -mlarge-data -mconst-in-data -O1 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o: hardware_initializers/pic24fj64ga004_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/pic24fj64ga004_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o.d"        -g -omf=elf -ffunction-sections -fdata-sections -mlarge-code -mlarge-data -mconst-in-data -O1 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o: hardware_initializers/dspic33ep256gp506_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/dspic33ep256gp506_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o.d"        -g -omf=elf -ffunction-sections -fdata-sections -mlarge-code -mlarge-data -mconst-in-data -O1 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o: hardware_initializers/dspic33fj256gp710a_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/dspic33fj256gp710a_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o.d"        -g -omf=elf -ffunction-sections -fdata-sections -mlarge-code -mlarge-data -mconst-in-data -O1 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o: hardware_initializers/pic24fj1024gb610_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/pic24fj1024gb610_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o.d"        -g -omf=elf -ffunction-sections -fdata-sections -mlarge-code -mlarge-data -mconst-in-data -O1 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main.c  -o ${OBJECTDIR}/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/main.o.d"        -g -omf=elf -ffunction-sections -fdata-sections -mlarge-code -mlarge-data -mconst-in-data -O1 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/now.o: now.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/now.o.d 
	@${RM} ${OBJECTDIR}/now.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  now.c  -o ${OBJECTDIR}/now.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/now.o.d"        -g -omf=elf -ffunction-sections -fdata-sections -mlarge-code -mlarge-data -mconst-in-data -O1 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/now.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/i2c_fifo.o: i2c_fifo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/i2c_fifo.o.d 
	@${RM} ${OBJECTDIR}/i2c_fifo.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  i2c_fifo.c  -o ${OBJECTDIR}/i2c_fifo.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/i2c_fifo.o.d"        -g -omf=elf -ffunction-sections -fdata-sections -mlarge-code -mlarge-data -mconst-in-data -O1 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/i2c_fifo.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/ex_boot_i2c.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    ezbl_integration/ezbl_build_standalone.gld
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
###<EZBL MODIFIED 0>###
	@echo EZBL: Updating automatic sections in linker script
	${MP_JAVA_PATH}java -cp "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_tools.jar;${PATH_TO_IDE_BIN}../mplablibs/modules/ext/crownking.edc.jar;${PATH_TO_IDE_BIN}../mplablibs/modules/ext/crownking.jar;${PATH_TO_IDE_BIN}../mplablibs/modules/ext/jna.jar" com.microchip.apps.ezbl.Main --gldbuilder  -conf=${CONF} -mcpu=${MP_PROCESSOR_OPTION} -artifact="${FINAL_IMAGE}" -linkscript=${MP_LINKER_FILE_OPTION} -compiler_folder=${MP_CC_DIR} -java=${MP_JAVA_PATH} -dist_dir="${DISTDIR}"
	@echo EZBL: Starting linking pass 1 of 2
	${MP_CC} $(MP_EXTRA_LD_PRE) -o "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_partial_link_temp.elf"  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -omf=elf     -Wl,,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_SIMULATOR=1,$(MP_LINKER_FILE_OPTION),--stack=16,--no-check-sections,--data-init,--pack-data,--handles,--no-isr,--gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST)  1> NUL
	${MP_CC_DIR}\xc16-objdump -x -t -r "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_partial_link_temp.elf" > "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_partial_link_symbols.txt"
	${MP_CC_DIR}\xc16-objdump -s "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_partial_link_temp.elf" > "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_partial_link_sections.txt"
	${MP_JAVA_PATH}java -cp "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_tools.jar;${PATH_TO_IDE_BIN}../mplablibs/modules/ext/crownking.edc.jar;${PATH_TO_IDE_BIN}../mplablibs/modules/ext/crownking.jar;${PATH_TO_IDE_BIN}../mplablibs/modules/ext/jna.jar" com.microchip.apps.ezbl.Main --dump_parser -symbol_dump="C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_partial_link_symbols.txt" -section_dump="C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_partial_link_sections.txt" -conf=${CONF} -mcpu=${MP_PROCESSOR_OPTION} -artifact="${FINAL_IMAGE}" -linkscript=${MP_LINKER_FILE_OPTION} -compiler_folder=${MP_CC_DIR} -java=${MP_JAVA_PATH} -dist_dir="${DISTDIR}"
	@rm "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_partial_link_temp.elf"
	@rm "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_partial_link_symbols.txt"
	@rm "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_partial_link_sections.txt"
	@echo EZBL: Starting linking pass 2 of 2
###</EZBL MODIFIED 0>###
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/ex_boot_i2c.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG -D__MPLAB_DEBUGGER_SIMULATOR=1  -omf=elf     -Wl,,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_SIMULATOR=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--no-isr,--gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST) 
###<EZBL MODIFIED 0>###
#	Run the ezbl_tools.jar dump processor again so we can extract the final EZBL reserved locations and use them later to omit the EZBL code during .blob creation or whatnot
	${MP_CC_DIR}\xc16-objdump -x -t -r "dist/${CND_CONF}/${IMAGE_TYPE}/ex_boot_i2c.${IMAGE_TYPE}.${OUTPUT_SUFFIX}" > "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_link_symbols.txt"
	${MP_CC_DIR}\xc16-objdump -s "dist/${CND_CONF}/${IMAGE_TYPE}/ex_boot_i2c.${IMAGE_TYPE}.${OUTPUT_SUFFIX}" > "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_link_sections.txt"
	${MP_JAVA_PATH}java -cp "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_tools.jar;${PATH_TO_IDE_BIN}../mplablibs/modules/ext/crownking.edc.jar;${PATH_TO_IDE_BIN}../mplablibs/modules/ext/crownking.jar;${PATH_TO_IDE_BIN}../mplablibs/modules/ext/jna.jar" com.microchip.apps.ezbl.Main --dump_parser -symbol_dump="C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_link_symbols.txt" -section_dump="C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_link_sections.txt" -conf=${CONF} -mcpu=${MP_PROCESSOR_OPTION} -artifact="${FINAL_IMAGE}" -linkscript=${MP_LINKER_FILE_OPTION} -compiler_folder=${MP_CC_DIR} -java=${MP_JAVA_PATH} -dist_dir="${DISTDIR}"
	@rm "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_link_symbols.txt"
	@rm "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_link_sections.txt"
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
	@echo Converting debug elf to hex
	${MP_CC_DIR}\\xc16-bin2hex ${FINAL_IMAGE} -a -omf=elf
###<EZBL MODIFIED 1>###
	@echo Converting ${CND_ARTIFACT_NAME_${CONF}} to an EZBL blob
	${MP_JAVA_PATH}java -cp "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_tools.jar;${PATH_TO_IDE_BIN}../mplablibs/modules/ext/crownking.edc.jar;${PATH_TO_IDE_BIN}../mplablibs/modules/ext/crownking.jar;${PATH_TO_IDE_BIN}../mplablibs/modules/ext/jna.jar" com.microchip.apps.ezbl.Main --blobber  -conf=${CONF} -mcpu=${MP_PROCESSOR_OPTION} -artifact="$(subst .elf,.hex,${FINAL_IMAGE})" -linkscript=${MP_LINKER_FILE_OPTION} -compiler_folder=${MP_CC_DIR} -java=${MP_JAVA_PATH} -dist_dir="${DISTDIR}" -generate_merge
###</EZBL MODIFIED 1>###
endif
###</EZBL MODIFIED 0>###
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/ex_boot_i2c.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   ezbl_integration/ezbl_build_standalone.gld
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
###<EZBL MODIFIED 0>###
	@echo EZBL: Updating automatic sections in linker script
	${MP_JAVA_PATH}java -cp "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_tools.jar;${PATH_TO_IDE_BIN}../mplablibs/modules/ext/crownking.edc.jar;${PATH_TO_IDE_BIN}../mplablibs/modules/ext/crownking.jar;${PATH_TO_IDE_BIN}../mplablibs/modules/ext/jna.jar" com.microchip.apps.ezbl.Main --gldbuilder  -conf=${CONF} -mcpu=${MP_PROCESSOR_OPTION} -artifact="${FINAL_IMAGE}" -linkscript=${MP_LINKER_FILE_OPTION} -compiler_folder=${MP_CC_DIR} -java=${MP_JAVA_PATH} -dist_dir="${DISTDIR}"
	@echo EZBL: Starting linking pass 1 of 2
	${MP_CC} $(MP_EXTRA_LD_PRE) -o "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_partial_link_temp.elf"  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -Wl,,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--no-check-sections,--data-init,--pack-data,--handles,--no-isr,--gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST)  1> NUL
	${MP_CC_DIR}\xc16-objdump -x -t -r "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_partial_link_temp.elf" > "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_partial_link_symbols.txt"
	${MP_CC_DIR}\xc16-objdump -s "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_partial_link_temp.elf" > "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_partial_link_sections.txt"
	${MP_JAVA_PATH}java -cp "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_tools.jar;${PATH_TO_IDE_BIN}../mplablibs/modules/ext/crownking.edc.jar;${PATH_TO_IDE_BIN}../mplablibs/modules/ext/crownking.jar;${PATH_TO_IDE_BIN}../mplablibs/modules/ext/jna.jar" com.microchip.apps.ezbl.Main --dump_parser -symbol_dump="C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_partial_link_symbols.txt" -section_dump="C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_partial_link_sections.txt" -conf=${CONF} -mcpu=${MP_PROCESSOR_OPTION} -artifact="${FINAL_IMAGE}" -linkscript=${MP_LINKER_FILE_OPTION} -compiler_folder=${MP_CC_DIR} -java=${MP_JAVA_PATH} -dist_dir="${DISTDIR}"
	@rm "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_partial_link_temp.elf"
	@rm "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_partial_link_symbols.txt"
	@rm "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_partial_link_sections.txt"
	@echo EZBL: Starting linking pass 2 of 2
###</EZBL MODIFIED 0>###
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/ex_boot_i2c.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -Wl,,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--no-isr,--gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST) 
###<EZBL MODIFIED 0>###
#	Run the ezbl_tools.jar dump processor again so we can extract the final EZBL reserved locations and use them later to omit the EZBL code during .blob creation or whatnot
	${MP_CC_DIR}\xc16-objdump -x -t -r "dist/${CND_CONF}/${IMAGE_TYPE}/ex_boot_i2c.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}" > "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_link_symbols.txt"
	${MP_CC_DIR}\xc16-objdump -s "dist/${CND_CONF}/${IMAGE_TYPE}/ex_boot_i2c.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}" > "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_link_sections.txt"
	${MP_JAVA_PATH}java -cp "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_tools.jar;${PATH_TO_IDE_BIN}../mplablibs/modules/ext/crownking.edc.jar;${PATH_TO_IDE_BIN}../mplablibs/modules/ext/crownking.jar;${PATH_TO_IDE_BIN}../mplablibs/modules/ext/jna.jar" com.microchip.apps.ezbl.Main --dump_parser -symbol_dump="C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_link_symbols.txt" -section_dump="C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_link_sections.txt" -conf=${CONF} -mcpu=${MP_PROCESSOR_OPTION} -artifact="${FINAL_IMAGE}" -linkscript=${MP_LINKER_FILE_OPTION} -compiler_folder=${MP_CC_DIR} -java=${MP_JAVA_PATH} -dist_dir="${DISTDIR}"
	@rm "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_link_symbols.txt"
	@rm "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_link_sections.txt"
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
	@echo Converting debug elf to hex
	${MP_CC_DIR}\\xc16-bin2hex ${FINAL_IMAGE} -a -omf=elf
###<EZBL MODIFIED 1>###
	@echo Converting ${CND_ARTIFACT_NAME_${CONF}} to an EZBL blob
	${MP_JAVA_PATH}java -cp "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_tools.jar;${PATH_TO_IDE_BIN}../mplablibs/modules/ext/crownking.edc.jar;${PATH_TO_IDE_BIN}../mplablibs/modules/ext/crownking.jar;${PATH_TO_IDE_BIN}../mplablibs/modules/ext/jna.jar" com.microchip.apps.ezbl.Main --blobber  -conf=${CONF} -mcpu=${MP_PROCESSOR_OPTION} -artifact="$(subst .elf,.hex,${FINAL_IMAGE})" -linkscript=${MP_LINKER_FILE_OPTION} -compiler_folder=${MP_CC_DIR} -java=${MP_JAVA_PATH} -dist_dir="${DISTDIR}" -generate_merge
###</EZBL MODIFIED 1>###
endif
###</EZBL MODIFIED 0>###
	${MP_CC_DIR}\\xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/ex_boot_i2c.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf  
###<EZBL MODIFIED 1>###
	@echo Converting ${CND_ARTIFACT_NAME_${CONF}} to an EZBL blob
	${MP_JAVA_PATH}java -cp "C:\Work\ezbl\ex_boot_i2c\ezbl_integration\ezbl_tools.jar;${PATH_TO_IDE_BIN}../mplablibs/modules/ext/crownking.edc.jar;${PATH_TO_IDE_BIN}../mplablibs/modules/ext/crownking.jar;${PATH_TO_IDE_BIN}../mplablibs/modules/ext/jna.jar" com.microchip.apps.ezbl.Main --blobber  -conf=${CONF} -mcpu=${MP_PROCESSOR_OPTION} -artifact="$(subst .elf,.hex,${FINAL_IMAGE})" -linkscript=${MP_LINKER_FILE_OPTION} -compiler_folder=${MP_CC_DIR} -java=${MP_JAVA_PATH} -dist_dir="${DISTDIR}" -generate_merge
###</EZBL MODIFIED 1>###
	
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
