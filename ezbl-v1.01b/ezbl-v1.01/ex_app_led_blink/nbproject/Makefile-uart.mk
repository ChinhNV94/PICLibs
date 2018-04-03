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
ifeq "$(wildcard nbproject/Makefile-local-uart.mk)" "nbproject/Makefile-local-uart.mk"
include nbproject/Makefile-local-uart.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=uart
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/ex_app_led_blink.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/ex_app_led_blink.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=hardware_initializers/dspic33ep512gm710_explorer_16.c hardware_initializers/dspic33ep512mu810_explorer_16.c hardware_initializers/pic24fj128ga010_explorer_16.c hardware_initializers/pic24fj256ga110_explorer_16.c hardware_initializers/pic24fj256gb110_explorer_16.c hardware_initializers/pic24fj64ga004_explorer_16.c hardware_initializers/dspic33ep256gp506_explorer_16.c hardware_initializers/dspic33fj256gp710a_explorer_16.c hardware_initializers/pic24fj1024gb610_explorer_16.c main.c ezbl_integration/ex_boot_uart.merge.s

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o ${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o ${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o ${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o ${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o ${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o ${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o ${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o ${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o ${OBJECTDIR}/main.o ${OBJECTDIR}/ezbl_integration/ex_boot_uart.merge.o
POSSIBLE_DEPFILES=${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o.d ${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o.d ${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o.d ${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o.d ${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o.d ${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o.d ${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o.d ${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o.d ${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o.d ${OBJECTDIR}/main.o.d ${OBJECTDIR}/ezbl_integration/ex_boot_uart.merge.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o ${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o ${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o ${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o ${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o ${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o ${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o ${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o ${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o ${OBJECTDIR}/main.o ${OBJECTDIR}/ezbl_integration/ex_boot_uart.merge.o

# Source Files
SOURCEFILES=hardware_initializers/dspic33ep512gm710_explorer_16.c hardware_initializers/dspic33ep512mu810_explorer_16.c hardware_initializers/pic24fj128ga010_explorer_16.c hardware_initializers/pic24fj256ga110_explorer_16.c hardware_initializers/pic24fj256gb110_explorer_16.c hardware_initializers/pic24fj64ga004_explorer_16.c hardware_initializers/dspic33ep256gp506_explorer_16.c hardware_initializers/dspic33fj256gp710a_explorer_16.c hardware_initializers/pic24fj1024gb610_explorer_16.c main.c ezbl_integration/ex_boot_uart.merge.s


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
	${MAKE}  -f nbproject/Makefile-uart.mk dist/${CND_CONF}/${IMAGE_TYPE}/ex_app_led_blink.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33EP512MU810
MP_LINKER_FILE_OPTION=,--script=ezbl_integration/ex_boot_uart.merge.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o: hardware_initializers/dspic33ep512gm710_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/dspic33ep512gm710_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -mno-eds-warn  -omf=elf -mlarge-code -mlarge-data -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o: hardware_initializers/dspic33ep512mu810_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/dspic33ep512mu810_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -mno-eds-warn  -omf=elf -mlarge-code -mlarge-data -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o: hardware_initializers/pic24fj128ga010_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/pic24fj128ga010_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -mno-eds-warn  -omf=elf -mlarge-code -mlarge-data -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o: hardware_initializers/pic24fj256ga110_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/pic24fj256ga110_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -mno-eds-warn  -omf=elf -mlarge-code -mlarge-data -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o: hardware_initializers/pic24fj256gb110_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/pic24fj256gb110_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -mno-eds-warn  -omf=elf -mlarge-code -mlarge-data -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o: hardware_initializers/pic24fj64ga004_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/pic24fj64ga004_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -mno-eds-warn  -omf=elf -mlarge-code -mlarge-data -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o: hardware_initializers/dspic33ep256gp506_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/dspic33ep256gp506_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -mno-eds-warn  -omf=elf -mlarge-code -mlarge-data -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o: hardware_initializers/dspic33fj256gp710a_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/dspic33fj256gp710a_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -mno-eds-warn  -omf=elf -mlarge-code -mlarge-data -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o: hardware_initializers/pic24fj1024gb610_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/pic24fj1024gb610_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -mno-eds-warn  -omf=elf -mlarge-code -mlarge-data -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main.c  -o ${OBJECTDIR}/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/main.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -mno-eds-warn  -omf=elf -mlarge-code -mlarge-data -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o: hardware_initializers/dspic33ep512gm710_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/dspic33ep512gm710_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o.d"      -mno-eds-warn  -g -omf=elf -mlarge-code -mlarge-data -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/dspic33ep512gm710_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o: hardware_initializers/dspic33ep512mu810_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/dspic33ep512mu810_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o.d"      -mno-eds-warn  -g -omf=elf -mlarge-code -mlarge-data -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/dspic33ep512mu810_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o: hardware_initializers/pic24fj128ga010_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/pic24fj128ga010_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o.d"      -mno-eds-warn  -g -omf=elf -mlarge-code -mlarge-data -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/pic24fj128ga010_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o: hardware_initializers/pic24fj256ga110_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/pic24fj256ga110_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o.d"      -mno-eds-warn  -g -omf=elf -mlarge-code -mlarge-data -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/pic24fj256ga110_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o: hardware_initializers/pic24fj256gb110_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/pic24fj256gb110_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o.d"      -mno-eds-warn  -g -omf=elf -mlarge-code -mlarge-data -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/pic24fj256gb110_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o: hardware_initializers/pic24fj64ga004_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/pic24fj64ga004_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o.d"      -mno-eds-warn  -g -omf=elf -mlarge-code -mlarge-data -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/pic24fj64ga004_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o: hardware_initializers/dspic33ep256gp506_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/dspic33ep256gp506_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o.d"      -mno-eds-warn  -g -omf=elf -mlarge-code -mlarge-data -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/dspic33ep256gp506_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o: hardware_initializers/dspic33fj256gp710a_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/dspic33fj256gp710a_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o.d"      -mno-eds-warn  -g -omf=elf -mlarge-code -mlarge-data -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/dspic33fj256gp710a_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o: hardware_initializers/pic24fj1024gb610_explorer_16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/hardware_initializers" 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o.d 
	@${RM} ${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  hardware_initializers/pic24fj1024gb610_explorer_16.c  -o ${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o.d"      -mno-eds-warn  -g -omf=elf -mlarge-code -mlarge-data -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/hardware_initializers/pic24fj1024gb610_explorer_16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main.c  -o ${OBJECTDIR}/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/main.o.d"      -mno-eds-warn  -g -omf=elf -mlarge-code -mlarge-data -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/ezbl_integration/ex_boot_uart.merge.o: ezbl_integration/ex_boot_uart.merge.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/ezbl_integration" 
	@${RM} ${OBJECTDIR}/ezbl_integration/ex_boot_uart.merge.o.d 
	@${RM} ${OBJECTDIR}/ezbl_integration/ex_boot_uart.merge.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ezbl_integration/ex_boot_uart.merge.s  -o ${OBJECTDIR}/ezbl_integration/ex_boot_uart.merge.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -omf=elf -Wa,-MD,"${OBJECTDIR}/ezbl_integration/ex_boot_uart.merge.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_REAL_ICE=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/ezbl_integration/ex_boot_uart.merge.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
else
${OBJECTDIR}/ezbl_integration/ex_boot_uart.merge.o: ezbl_integration/ex_boot_uart.merge.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/ezbl_integration" 
	@${RM} ${OBJECTDIR}/ezbl_integration/ex_boot_uart.merge.o.d 
	@${RM} ${OBJECTDIR}/ezbl_integration/ex_boot_uart.merge.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ezbl_integration/ex_boot_uart.merge.s  -o ${OBJECTDIR}/ezbl_integration/ex_boot_uart.merge.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -Wa,-MD,"${OBJECTDIR}/ezbl_integration/ex_boot_uart.merge.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/ezbl_integration/ex_boot_uart.merge.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/ex_app_led_blink.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    ezbl_integration/ex_boot_uart.merge.gld
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/ex_app_led_blink.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG -D__MPLAB_DEBUGGER_REAL_ICE=1  -omf=elf  -mreserve=data@0x1000:0x101B -mreserve=data@0x101C:0x101D -mreserve=data@0x101E:0x101F -mreserve=data@0x1020:0x1021 -mreserve=data@0x1022:0x1023 -mreserve=data@0x1024:0x1027 -mreserve=data@0x1028:0x104F   -Wl,--local-stack,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_REAL_ICE=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST) 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/ex_app_led_blink.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   ezbl_integration/ex_boot_uart.merge.gld
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/ex_app_led_blink.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -Wl,--local-stack,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST) 
	${MP_CC_DIR}\\xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/ex_app_led_blink.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf  
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/uart
	${RM} -r dist/uart

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
