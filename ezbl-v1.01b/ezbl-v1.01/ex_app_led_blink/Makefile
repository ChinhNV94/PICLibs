#
#  There exist several targets which are by default empty and which can be 
#  used for execution of your targets. These targets are usually executed 
#  before and after some main targets. They are: 
#
#     .build-pre:              called before 'build' target
#     .build-post:             called after 'build' target
#     .clean-pre:              called before 'clean' target
#     .clean-post:             called after 'clean' target
#     .clobber-pre:            called before 'clobber' target
#     .clobber-post:           called after 'clobber' target
#     .all-pre:                called before 'all' target
#     .all-post:               called after 'all' target
#     .help-pre:               called before 'help' target
#     .help-post:              called after 'help' target
#
#  Targets beginning with '.' are not intended to be called on their own.
#
#  Main targets can be executed directly, and they are:
#  
#     build                    build a specific configuration
#     clean                    remove built files from a configuration
#     clobber                  remove all built files
#     all                      build all configurations
#     help                     print help mesage
#  
#  Targets .build-impl, .clean-impl, .clobber-impl, .all-impl, and
#  .help-impl are implemented in nbproject/makefile-impl.mk.
#
#  Available make variables:
#
#     CND_BASEDIR                base directory for relative paths
#     CND_DISTDIR                default top distribution directory (build artifacts)
#     CND_BUILDDIR               default top build directory (object files, ...)
#     CONF                       name of current configuration
#     CND_ARTIFACT_DIR_${CONF}   directory of build artifact (current configuration)
#     CND_ARTIFACT_NAME_${CONF}  name of build artifact (current configuration)
#     CND_ARTIFACT_PATH_${CONF}  path to build artifact (current configuration)
#     CND_PACKAGE_DIR_${CONF}    directory of package (current configuration)
#     CND_PACKAGE_NAME_${CONF}   name of package (current configuration)
#     CND_PACKAGE_PATH_${CONF}   path to package (current configuration)
#
# NOCDDL


# Environment 
MKDIR=mkdir
CP=cp
CCADMIN=CCadmin
RANLIB=ranlib

# Suppress make[x] Entering directory messages and try to speed up make by 
# telling it not to consider any builtin make rules. MPLAB X generates makefiles 
# that are fully explicit, so there is no need for implicit rule consideration.
MAKEFLAGS += --no-print-directory --no-builtin-rules



# build
build: .build-post


.build-pre:
# Add your pre 'build' code here...


.build-post: .build-impl
# Add your post 'build' code here...
	@echo EZBL: Converting .hex file to a binary .blob
	${MP_JAVA_PATH}java -jar "ezbl_integration/ezbl_tools.jar" --blobber -artifact="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.hex"
	@echo EZBL: Attempting to send blob to bootloader

ifeq "${CONF}" "uart"	# Check if "uart" MPLAB project build profile is used
	${MP_JAVA_PATH}java -jar "ezbl_integration/ezbl_tools.jar" --communicator -com=\\\\.\\COM8 -baud=230400 -timeout=1000 -artifact="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.blob"
else	# Not "uart" build profile; must be "i2c" project being built
	${MP_JAVA_PATH}java -jar "ezbl_integration/ezbl_tools.jar" --communicator -com=I2C -slave_address=0x60 -baud=400000 -timeout=1000 -artifact="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.blob"
endif


# clean
clean: .clean-post

.clean-pre:
# Add your pre 'clean' code here...
# WARNING: the IDE does not call this target since it takes a long time to
# simply run make. Instead, the IDE removes the configuration directories
# under build and dist directly without calling make.
# This target is left here so people can do a clean when running a clean
# outside the IDE.

.clean-post: .clean-impl
# Add your post 'clean' code here...


# clobber
clobber: .clobber-post

.clobber-pre:
# Add your pre 'clobber' code here...

.clobber-post: .clobber-impl
# Add your post 'clobber' code here...


# all
all: .all-post

.all-pre:
# Add your pre 'all' code here...

.all-post: .all-impl
# Add your post 'all' code here...


# help
help: .help-post

.help-pre:
# Add your pre 'help' code here...

.help-post: .help-impl
# Add your post 'help' code here...



# include project implementation makefile
include nbproject/Makefile-impl.mk

# include project make variables
include nbproject/Makefile-variables.mk


# Test if Makefile-${CONF}.mk has already been read in, and if so, don't read it 
# in again (doing so would make duplicate rules and associated recipes whenever 
# you invoke a 'Compile' or 'Assemble' on a single source file).
ifeq (,$(findstring nbproject/Makefile-${CONF}.mk,$(MAKEFILE_LIST)))

# Obtain paths for MPLAB executables (ex: ${MP_JAVA_PATH} variable)
include nbproject/Makefile-local-${CONF}.mk

# include the meat of this configuration's makefile that has all the useful
# MPLAB project settings assigned to variables (ex: ${MP_PROCESSOR_OPTION},
# ${DISTDIR}, etc.). Directly including this file will not work since it in turn 
# includes Makefile (this file), which would set up a circular reference and 
# redefine all recipes. Setting IGNORE_LOCAL = TRUE fixes this, but since it 
# also has the repercussion of losing access to Makefile-local-${CONF}.mk 
# definitions, we can correct this by just exporting all variables so they 
# automatically propagate for use when executing the recipes.
IGNORE_LOCAL = TRUE
export
include nbproject/Makefile-${CONF}.mk
endif	# ifeq duplicate rule and recipe suppression