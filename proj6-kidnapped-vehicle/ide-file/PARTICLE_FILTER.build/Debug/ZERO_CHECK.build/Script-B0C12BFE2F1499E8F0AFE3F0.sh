#!/bin/sh
set -e
if test "$CONFIGURATION" = "Debug"; then :
  cd /Users/fanye/WorkSpace/Udacity/sdc-nano/projs/proj6-kidnapped-vehicle/ide-file
  make -f /Users/fanye/WorkSpace/Udacity/sdc-nano/projs/proj6-kidnapped-vehicle/ide-file/CMakeScripts/ReRunCMake.make
fi
if test "$CONFIGURATION" = "Release"; then :
  cd /Users/fanye/WorkSpace/Udacity/sdc-nano/projs/proj6-kidnapped-vehicle/ide-file
  make -f /Users/fanye/WorkSpace/Udacity/sdc-nano/projs/proj6-kidnapped-vehicle/ide-file/CMakeScripts/ReRunCMake.make
fi
if test "$CONFIGURATION" = "MinSizeRel"; then :
  cd /Users/fanye/WorkSpace/Udacity/sdc-nano/projs/proj6-kidnapped-vehicle/ide-file
  make -f /Users/fanye/WorkSpace/Udacity/sdc-nano/projs/proj6-kidnapped-vehicle/ide-file/CMakeScripts/ReRunCMake.make
fi
if test "$CONFIGURATION" = "RelWithDebInfo"; then :
  cd /Users/fanye/WorkSpace/Udacity/sdc-nano/projs/proj6-kidnapped-vehicle/ide-file
  make -f /Users/fanye/WorkSpace/Udacity/sdc-nano/projs/proj6-kidnapped-vehicle/ide-file/CMakeScripts/ReRunCMake.make
fi

