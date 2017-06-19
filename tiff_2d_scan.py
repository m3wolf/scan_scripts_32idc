'''
	Tiff 2d for Sector 32 ID C

'''
import sys
import json
import time
from epics import PV
import h5py
import shutil
import os
import imp
import traceback

from tomo_scan_lib import *


global variableDict

variableDict = {'PreDarkImages': 0,
				'PreWhiteImages': 0,
				'Projections': 100000000,
				'SampleXOut': 0.0,
				'SampleYOut': 0.0,
				'SampleZOut': 0.0,
				'SampleXIn': 0.0,
				'SampleYIn': 0.0,
				'SampleZIn': 0.0,
				'StartSleep_min': 0,
				'ExposureTime': 1,
				'ExternalShutter': 0,
				'Ext_ShutterOpenDelay': 0.04,
				'Delay_next_exposure': 3,
				'IOC_Prefix': '32idcPG3:',
				'TIFFNDArrayPort': 'PROC1',
				'FileWriteMode': 'Single',
				'Recursive_Filter_Enabled': 0,
				'Recursive_Filter_N_Images': 2,
				'Recursive_Filter_Type': 'RecursiveAve'
				}

global_PVs = {}

def getVariableDict():
	global variableDict
	return variableDict

def getVariableDict():
	return variableDict

def tiff_scan():
	print 'tiff_scan()'
#	global_PVs['Cam1_FrameType'].put(FrameTypeData, wait=True)
	global_PVs['Cam1_NumImages'].put(1, wait=True)
	if variableDict['Recursive_Filter_Enabled'] == 1:
		global_PVs['Proc1_Filter_Enable'].put('Enable')

	for i in range(int(variableDict['Projections'])):
		if variableDict['Recursive_Filter_Enabled'] == 1:
			global_PVs['Proc1_Callbacks'].put('Enable', wait=True)
			for k in range(int(variableDict['Recursive_Filter_N_Images'])):
				global_PVs['Cam1_Acquire'].put(DetectorAcquire)
				wait_pv(global_PVs['Cam1_Acquire'], DetectorAcquire, 2)
				global_PVs['Cam1_SoftwareTrigger'].put(1)
				wait_pv(global_PVs['Cam1_Acquire'], DetectorIdle, 60)
		else:
			global_PVs['Cam1_Acquire'].put(DetectorAcquire)
			wait_pv(global_PVs['Cam1_Acquire'], DetectorAcquire, 2)
			
			global_PVs['Fast_Shutter_Uniblitz'].put(1, wait=True) # open the fast shutter #### FOR SUJI
			time.sleep(int(variableDict['Ext_ShutterOpenDelay'])) #### FOR SUJI
			
			global_PVs['Cam1_SoftwareTrigger'].put(1)
		wait_pv(global_PVs['Cam1_Acquire'], DetectorIdle, 60)

		time.sleep(0.04) # Short pause before closing the fast shutter after acquisitions #### FOR SUJI @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
		global_PVs['Fast_Shutter_Uniblitz'].put(0, wait=True) # Close the fast shutter #### FOR SUJI
		time.sleep(float(variableDict['Delay_next_exposure'])) # Pause between 2 acquisition with shutter closed #### FOR SUJI

	if variableDict['Recursive_Filter_Enabled'] == 1:
		global_PVs['Proc1_Filter_Enable'].put('Disable', wait=True)

	return


def start_scan():
	print 'start_scan()'
	init_general_PVs(global_PVs, variableDict)
    	if variableDict.has_key('StopTheScan'): # stopping the scan in a clean way
		stop_scan(global_PVs, variableDict)
		return
	setup_detector(global_PVs, variableDict)
	setup_tiff_writer(global_PVs, variableDict)
	if int(variableDict['PreDarkImages']) > 0:
		close_shutters(global_PVs, variableDict)
		print 'Capturing Pre Dark Field'
		FileName = global_PVs['TIFF1_FileName'].get(as_string=True)
		global_PVs['TIFF1_FileName'].put(FileName+'_dark')
		capture_multiple_projections(global_PVs, variableDict, int(variableDict['PreDarkImages']), FrameTypeDark)
		global_PVs['TIFF1_FileName'].put(FileName)
	if int(variableDict['PreWhiteImages']) > 0:
		print 'Capturing Pre White Field'
		open_shutters(global_PVs, variableDict)
		move_sample_out(global_PVs, variableDict)
		FileName = global_PVs['TIFF1_FileName'].get(as_string=True)
		global_PVs['TIFF1_FileName'].put(FileName+'_white')
		capture_multiple_projections(global_PVs, variableDict, int(variableDict['PreWhiteImages']), FrameTypeWhite)
		global_PVs['TIFF1_FileName'].put(FileName)
	move_sample_in(global_PVs, variableDict)
	open_shutters(global_PVs, variableDict)
    
    # Main scan:
	tiff_scan()

	global_PVs['TIFF1_AutoSave'].put('No')
	global_PVs['TIFF1_Capture'].put(0)
	reset_CCD(global_PVs, variableDict)

def main():
	update_variable_dict(variableDict)
	start_scan()

if __name__ == '__main__':
	main()

