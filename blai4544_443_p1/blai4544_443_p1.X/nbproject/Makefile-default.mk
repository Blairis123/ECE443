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
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/blai4544_443_p1.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/blai4544_443_p1.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS

else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../../../FreeRTOS/Source/list.c ../../../FreeRTOS/Source/queue.c ../../../FreeRTOS/Source/tasks.c ../../../FreeRTOS/Source/portable/MPLAB/PIC32MX/port.c ../../../FreeRTOS/Source/portable/MPLAB/PIC32MX/port_asm.S ../../../FreeRTOS/Source/portable/MemMang/heap_4.c ../CerebotMX7cK.c ../main.c ../../../TraceRecorder/trcKernelPort.c ../../../TraceRecorder/trcSnapshotRecorder.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/190470153/list.o ${OBJECTDIR}/_ext/190470153/queue.o ${OBJECTDIR}/_ext/190470153/tasks.o ${OBJECTDIR}/_ext/1599763551/port.o ${OBJECTDIR}/_ext/1599763551/port_asm.o ${OBJECTDIR}/_ext/636803636/heap_4.o ${OBJECTDIR}/_ext/1472/CerebotMX7cK.o ${OBJECTDIR}/_ext/1472/main.o ${OBJECTDIR}/_ext/402131092/trcKernelPort.o ${OBJECTDIR}/_ext/402131092/trcSnapshotRecorder.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/190470153/list.o.d ${OBJECTDIR}/_ext/190470153/queue.o.d ${OBJECTDIR}/_ext/190470153/tasks.o.d ${OBJECTDIR}/_ext/1599763551/port.o.d ${OBJECTDIR}/_ext/1599763551/port_asm.o.d ${OBJECTDIR}/_ext/636803636/heap_4.o.d ${OBJECTDIR}/_ext/1472/CerebotMX7cK.o.d ${OBJECTDIR}/_ext/1472/main.o.d ${OBJECTDIR}/_ext/402131092/trcKernelPort.o.d ${OBJECTDIR}/_ext/402131092/trcSnapshotRecorder.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/190470153/list.o ${OBJECTDIR}/_ext/190470153/queue.o ${OBJECTDIR}/_ext/190470153/tasks.o ${OBJECTDIR}/_ext/1599763551/port.o ${OBJECTDIR}/_ext/1599763551/port_asm.o ${OBJECTDIR}/_ext/636803636/heap_4.o ${OBJECTDIR}/_ext/1472/CerebotMX7cK.o ${OBJECTDIR}/_ext/1472/main.o ${OBJECTDIR}/_ext/402131092/trcKernelPort.o ${OBJECTDIR}/_ext/402131092/trcSnapshotRecorder.o

# Source Files
SOURCEFILES=../../../FreeRTOS/Source/list.c ../../../FreeRTOS/Source/queue.c ../../../FreeRTOS/Source/tasks.c ../../../FreeRTOS/Source/portable/MPLAB/PIC32MX/port.c ../../../FreeRTOS/Source/portable/MPLAB/PIC32MX/port_asm.S ../../../FreeRTOS/Source/portable/MemMang/heap_4.c ../CerebotMX7cK.c ../main.c ../../../TraceRecorder/trcKernelPort.c ../../../TraceRecorder/trcSnapshotRecorder.c


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
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/blai4544_443_p1.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MX795F512L
MP_LINKER_FILE_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1599763551/port_asm.o: ../../../FreeRTOS/Source/portable/MPLAB/PIC32MX/port_asm.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1599763551" 
	@${RM} ${OBJECTDIR}/_ext/1599763551/port_asm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1599763551/port_asm.o 
	@${RM} ${OBJECTDIR}/_ext/1599763551/port_asm.o.ok ${OBJECTDIR}/_ext/1599763551/port_asm.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1599763551/port_asm.o.d" "${OBJECTDIR}/_ext/1599763551/port_asm.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../FreeRTOS/Source/portable/MPLAB/PIC32MX" -MMD -MF "${OBJECTDIR}/_ext/1599763551/port_asm.o.d"  -o ${OBJECTDIR}/_ext/1599763551/port_asm.o ../../../FreeRTOS/Source/portable/MPLAB/PIC32MX/port_asm.S  -DXPRJ_default=$(CND_CONF)    -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1599763551/port_asm.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1 -I ..
	
else
${OBJECTDIR}/_ext/1599763551/port_asm.o: ../../../FreeRTOS/Source/portable/MPLAB/PIC32MX/port_asm.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1599763551" 
	@${RM} ${OBJECTDIR}/_ext/1599763551/port_asm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1599763551/port_asm.o 
	@${RM} ${OBJECTDIR}/_ext/1599763551/port_asm.o.ok ${OBJECTDIR}/_ext/1599763551/port_asm.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1599763551/port_asm.o.d" "${OBJECTDIR}/_ext/1599763551/port_asm.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../FreeRTOS/Source/portable/MPLAB/PIC32MX" -MMD -MF "${OBJECTDIR}/_ext/1599763551/port_asm.o.d"  -o ${OBJECTDIR}/_ext/1599763551/port_asm.o ../../../FreeRTOS/Source/portable/MPLAB/PIC32MX/port_asm.S  -DXPRJ_default=$(CND_CONF)    -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1599763551/port_asm.o.asm.d",--gdwarf-2 -I ..
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/190470153/list.o: ../../../FreeRTOS/Source/list.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/190470153" 
	@${RM} ${OBJECTDIR}/_ext/190470153/list.o.d 
	@${RM} ${OBJECTDIR}/_ext/190470153/list.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/190470153/list.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -D_SUPRESS_PLIB_WARNING -D_DISABLE_OPENADC10_CONFIGPORT_WARNING -I".." -I"../../../FreeRTOS/Source/include" -I"../../../FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../TraceRecorder/include" -MMD -MF "${OBJECTDIR}/_ext/190470153/list.o.d" -o ${OBJECTDIR}/_ext/190470153/list.o ../../../FreeRTOS/Source/list.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/190470153/queue.o: ../../../FreeRTOS/Source/queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/190470153" 
	@${RM} ${OBJECTDIR}/_ext/190470153/queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/190470153/queue.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/190470153/queue.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -D_SUPRESS_PLIB_WARNING -D_DISABLE_OPENADC10_CONFIGPORT_WARNING -I".." -I"../../../FreeRTOS/Source/include" -I"../../../FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../TraceRecorder/include" -MMD -MF "${OBJECTDIR}/_ext/190470153/queue.o.d" -o ${OBJECTDIR}/_ext/190470153/queue.o ../../../FreeRTOS/Source/queue.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/190470153/tasks.o: ../../../FreeRTOS/Source/tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/190470153" 
	@${RM} ${OBJECTDIR}/_ext/190470153/tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/190470153/tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/190470153/tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -D_SUPRESS_PLIB_WARNING -D_DISABLE_OPENADC10_CONFIGPORT_WARNING -I".." -I"../../../FreeRTOS/Source/include" -I"../../../FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../TraceRecorder/include" -MMD -MF "${OBJECTDIR}/_ext/190470153/tasks.o.d" -o ${OBJECTDIR}/_ext/190470153/tasks.o ../../../FreeRTOS/Source/tasks.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1599763551/port.o: ../../../FreeRTOS/Source/portable/MPLAB/PIC32MX/port.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1599763551" 
	@${RM} ${OBJECTDIR}/_ext/1599763551/port.o.d 
	@${RM} ${OBJECTDIR}/_ext/1599763551/port.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1599763551/port.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -D_SUPRESS_PLIB_WARNING -D_DISABLE_OPENADC10_CONFIGPORT_WARNING -I".." -I"../../../FreeRTOS/Source/include" -I"../../../FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../TraceRecorder/include" -MMD -MF "${OBJECTDIR}/_ext/1599763551/port.o.d" -o ${OBJECTDIR}/_ext/1599763551/port.o ../../../FreeRTOS/Source/portable/MPLAB/PIC32MX/port.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/636803636/heap_4.o: ../../../FreeRTOS/Source/portable/MemMang/heap_4.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/636803636" 
	@${RM} ${OBJECTDIR}/_ext/636803636/heap_4.o.d 
	@${RM} ${OBJECTDIR}/_ext/636803636/heap_4.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/636803636/heap_4.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -D_SUPRESS_PLIB_WARNING -D_DISABLE_OPENADC10_CONFIGPORT_WARNING -I".." -I"../../../FreeRTOS/Source/include" -I"../../../FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../TraceRecorder/include" -MMD -MF "${OBJECTDIR}/_ext/636803636/heap_4.o.d" -o ${OBJECTDIR}/_ext/636803636/heap_4.o ../../../FreeRTOS/Source/portable/MemMang/heap_4.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1472/CerebotMX7cK.o: ../CerebotMX7cK.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/CerebotMX7cK.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/CerebotMX7cK.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/CerebotMX7cK.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -D_SUPRESS_PLIB_WARNING -D_DISABLE_OPENADC10_CONFIGPORT_WARNING -I".." -I"../../../FreeRTOS/Source/include" -I"../../../FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../TraceRecorder/include" -MMD -MF "${OBJECTDIR}/_ext/1472/CerebotMX7cK.o.d" -o ${OBJECTDIR}/_ext/1472/CerebotMX7cK.o ../CerebotMX7cK.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1472/main.o: ../main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -D_SUPRESS_PLIB_WARNING -D_DISABLE_OPENADC10_CONFIGPORT_WARNING -I".." -I"../../../FreeRTOS/Source/include" -I"../../../FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../TraceRecorder/include" -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d" -o ${OBJECTDIR}/_ext/1472/main.o ../main.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/402131092/trcKernelPort.o: ../../../TraceRecorder/trcKernelPort.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/402131092" 
	@${RM} ${OBJECTDIR}/_ext/402131092/trcKernelPort.o.d 
	@${RM} ${OBJECTDIR}/_ext/402131092/trcKernelPort.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/402131092/trcKernelPort.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -D_SUPRESS_PLIB_WARNING -D_DISABLE_OPENADC10_CONFIGPORT_WARNING -I".." -I"../../../FreeRTOS/Source/include" -I"../../../FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../TraceRecorder/include" -MMD -MF "${OBJECTDIR}/_ext/402131092/trcKernelPort.o.d" -o ${OBJECTDIR}/_ext/402131092/trcKernelPort.o ../../../TraceRecorder/trcKernelPort.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/402131092/trcSnapshotRecorder.o: ../../../TraceRecorder/trcSnapshotRecorder.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/402131092" 
	@${RM} ${OBJECTDIR}/_ext/402131092/trcSnapshotRecorder.o.d 
	@${RM} ${OBJECTDIR}/_ext/402131092/trcSnapshotRecorder.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/402131092/trcSnapshotRecorder.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -D_SUPRESS_PLIB_WARNING -D_DISABLE_OPENADC10_CONFIGPORT_WARNING -I".." -I"../../../FreeRTOS/Source/include" -I"../../../FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../TraceRecorder/include" -MMD -MF "${OBJECTDIR}/_ext/402131092/trcSnapshotRecorder.o.d" -o ${OBJECTDIR}/_ext/402131092/trcSnapshotRecorder.o ../../../TraceRecorder/trcSnapshotRecorder.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD) 
	
else
${OBJECTDIR}/_ext/190470153/list.o: ../../../FreeRTOS/Source/list.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/190470153" 
	@${RM} ${OBJECTDIR}/_ext/190470153/list.o.d 
	@${RM} ${OBJECTDIR}/_ext/190470153/list.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/190470153/list.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -D_SUPRESS_PLIB_WARNING -D_DISABLE_OPENADC10_CONFIGPORT_WARNING -I".." -I"../../../FreeRTOS/Source/include" -I"../../../FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../TraceRecorder/include" -MMD -MF "${OBJECTDIR}/_ext/190470153/list.o.d" -o ${OBJECTDIR}/_ext/190470153/list.o ../../../FreeRTOS/Source/list.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/190470153/queue.o: ../../../FreeRTOS/Source/queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/190470153" 
	@${RM} ${OBJECTDIR}/_ext/190470153/queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/190470153/queue.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/190470153/queue.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -D_SUPRESS_PLIB_WARNING -D_DISABLE_OPENADC10_CONFIGPORT_WARNING -I".." -I"../../../FreeRTOS/Source/include" -I"../../../FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../TraceRecorder/include" -MMD -MF "${OBJECTDIR}/_ext/190470153/queue.o.d" -o ${OBJECTDIR}/_ext/190470153/queue.o ../../../FreeRTOS/Source/queue.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/190470153/tasks.o: ../../../FreeRTOS/Source/tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/190470153" 
	@${RM} ${OBJECTDIR}/_ext/190470153/tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/190470153/tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/190470153/tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -D_SUPRESS_PLIB_WARNING -D_DISABLE_OPENADC10_CONFIGPORT_WARNING -I".." -I"../../../FreeRTOS/Source/include" -I"../../../FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../TraceRecorder/include" -MMD -MF "${OBJECTDIR}/_ext/190470153/tasks.o.d" -o ${OBJECTDIR}/_ext/190470153/tasks.o ../../../FreeRTOS/Source/tasks.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1599763551/port.o: ../../../FreeRTOS/Source/portable/MPLAB/PIC32MX/port.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1599763551" 
	@${RM} ${OBJECTDIR}/_ext/1599763551/port.o.d 
	@${RM} ${OBJECTDIR}/_ext/1599763551/port.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1599763551/port.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -D_SUPRESS_PLIB_WARNING -D_DISABLE_OPENADC10_CONFIGPORT_WARNING -I".." -I"../../../FreeRTOS/Source/include" -I"../../../FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../TraceRecorder/include" -MMD -MF "${OBJECTDIR}/_ext/1599763551/port.o.d" -o ${OBJECTDIR}/_ext/1599763551/port.o ../../../FreeRTOS/Source/portable/MPLAB/PIC32MX/port.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/636803636/heap_4.o: ../../../FreeRTOS/Source/portable/MemMang/heap_4.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/636803636" 
	@${RM} ${OBJECTDIR}/_ext/636803636/heap_4.o.d 
	@${RM} ${OBJECTDIR}/_ext/636803636/heap_4.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/636803636/heap_4.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -D_SUPRESS_PLIB_WARNING -D_DISABLE_OPENADC10_CONFIGPORT_WARNING -I".." -I"../../../FreeRTOS/Source/include" -I"../../../FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../TraceRecorder/include" -MMD -MF "${OBJECTDIR}/_ext/636803636/heap_4.o.d" -o ${OBJECTDIR}/_ext/636803636/heap_4.o ../../../FreeRTOS/Source/portable/MemMang/heap_4.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1472/CerebotMX7cK.o: ../CerebotMX7cK.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/CerebotMX7cK.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/CerebotMX7cK.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/CerebotMX7cK.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -D_SUPRESS_PLIB_WARNING -D_DISABLE_OPENADC10_CONFIGPORT_WARNING -I".." -I"../../../FreeRTOS/Source/include" -I"../../../FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../TraceRecorder/include" -MMD -MF "${OBJECTDIR}/_ext/1472/CerebotMX7cK.o.d" -o ${OBJECTDIR}/_ext/1472/CerebotMX7cK.o ../CerebotMX7cK.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1472/main.o: ../main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -D_SUPRESS_PLIB_WARNING -D_DISABLE_OPENADC10_CONFIGPORT_WARNING -I".." -I"../../../FreeRTOS/Source/include" -I"../../../FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../TraceRecorder/include" -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d" -o ${OBJECTDIR}/_ext/1472/main.o ../main.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/402131092/trcKernelPort.o: ../../../TraceRecorder/trcKernelPort.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/402131092" 
	@${RM} ${OBJECTDIR}/_ext/402131092/trcKernelPort.o.d 
	@${RM} ${OBJECTDIR}/_ext/402131092/trcKernelPort.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/402131092/trcKernelPort.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -D_SUPRESS_PLIB_WARNING -D_DISABLE_OPENADC10_CONFIGPORT_WARNING -I".." -I"../../../FreeRTOS/Source/include" -I"../../../FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../TraceRecorder/include" -MMD -MF "${OBJECTDIR}/_ext/402131092/trcKernelPort.o.d" -o ${OBJECTDIR}/_ext/402131092/trcKernelPort.o ../../../TraceRecorder/trcKernelPort.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/402131092/trcSnapshotRecorder.o: ../../../TraceRecorder/trcSnapshotRecorder.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/402131092" 
	@${RM} ${OBJECTDIR}/_ext/402131092/trcSnapshotRecorder.o.d 
	@${RM} ${OBJECTDIR}/_ext/402131092/trcSnapshotRecorder.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/402131092/trcSnapshotRecorder.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -D_SUPRESS_PLIB_WARNING -D_DISABLE_OPENADC10_CONFIGPORT_WARNING -I".." -I"../../../FreeRTOS/Source/include" -I"../../../FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../TraceRecorder/include" -MMD -MF "${OBJECTDIR}/_ext/402131092/trcSnapshotRecorder.o.d" -o ${OBJECTDIR}/_ext/402131092/trcSnapshotRecorder.o ../../../TraceRecorder/trcSnapshotRecorder.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD) 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/blai4544_443_p1.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mdebugger -D__MPLAB_DEBUGGER_PK3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/blai4544_443_p1.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)   -mreserve=data@0x0:0x1FC -mreserve=boot@0x1FC02000:0x1FC02FEF -mreserve=boot@0x1FC02000:0x1FC024FF  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map"
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/blai4544_443_p1.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/blai4544_443_p1.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map"
	${MP_CC_DIR}\\xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/blai4544_443_p1.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
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
