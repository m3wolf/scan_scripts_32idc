'''
	TomoScan for Sector 32 ID C

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
import numpy

from tomo_scan_lib import *

global variableDict

variableDict = {'ScanMotorName': 'zone_plate_2_z',
		'Motor_Start_Pos': -0.1,
		'Motor_End_Pos': 0.1,
		'Projections': 8,
		'PreDarkImages': 0,
		'PreWhiteImages': 0,
		'nImagesPerPos': 1, # saving several images / angle
		'PostDarkImages': 0,
		'PostWhiteImages': 0,
		'SampleXOut': 0.0,
#		'SampleYOut': 0.1,
#		'SampleZOut': 0,
#		'SampleRotOut': 90.0,
		'SampleXIn': 0.0,
#		'SampleYIn': 0.1,
#		'SampleZIn': 0.0,
		'StartSleep_min': 0,
		'StabilizeSleep_ms': 2000,
		'ExposureTime': 1,
		'IOC_Prefix': '32idcPG3:',
		'FileWriteMode': 'Single',
		'TIFFNDArrayPort': 'PROC1',
		'Recursive_Filter_Enabled': 1,
		'Recursive_Filter_N_Images': 10,
		'Recursive_Filter_Type': 'RecursiveAve',
		'Display_live': 1
		}

global_PVs = {}

def getVariableDict():
	global variableDict
	return variableDict

def getVariableDict():
	return variableDict

print('#######################################')
print('############ Starting Scan ############')
print('#######################################')

def tiff_scan():
	print 'tiff_scan()'

	Motor_Name = variableDict['ScanMotorName']
	print('*** Scanning ' + Motor_Name)

	vector_pos = numpy.linspace(float(variableDict['Motor_Start_Pos']), float(variableDict['Motor_End_Pos']), int(variableDict['Projections']))

	global_PVs['Cam1_FrameType'].put(FrameTypeData, wait=True)
	global_PVs['Cam1_NumImages'].put(1, wait=True)
	if variableDict['Recursive_Filter_Enabled'] == 1:
		global_PVs['Proc1_Filter_Enable'].put('Enable')

	for sample_pos in vector_pos:
		print('  '); print('  ### Motor position:', sample_pos); print('  ')
		global_PVs[Motor_Name].put(sample_pos, wait=True)
		time.sleep(float(variableDict['StabilizeSleep_ms'])/1000)

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
			global_PVs['Cam1_SoftwareTrigger'].put(1)
		wait_pv(global_PVs['Cam1_Acquire'], DetectorIdle, 60)
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
		global_PVs['TIFF1_FileName'].put(FileName+'_predark')
		capture_multiple_projections(global_PVs, variableDict, int(variableDict['PreDarkImages']), FrameTypeDark)
		global_PVs['TIFF1_FileName'].put(FileName)
	if int(variableDict['PreWhiteImages']) > 0:
		print 'Capturing Pre White Field'
		open_shutters(global_PVs, variableDict)
		move_sample_out(global_PVs, variableDict)
		FileName = global_PVs['TIFF1_FileName'].get(as_string=True)
		global_PVs['TIFF1_FileName'].put(FileName+'_prewhite')
		capture_multiple_projections(global_PVs, variableDict, int(variableDict['PreWhiteImages']), FrameTypeWhite)
		global_PVs['TIFF1_FileName'].put(FileName)
	move_sample_in(global_PVs, variableDict)
	open_shutters(global_PVs, variableDict)
    

    # Main scan:
    ####################################################
	tiff_scan()
    ####################################################


	# Post scan:
	if int(variableDict['PostWhiteImages']) > 0:
		print 'Capturing Post White Field'
		move_sample_out(global_PVs, variableDict)
		FileName = global_PVs['TIFF1_FileName'].get(as_string=True)
		global_PVs['TIFF1_FileName'].put(FileName+'_postwhite')
		capture_multiple_projections(global_PVs, variableDict, int(variableDict['PreWhiteImages']), FrameTypeWhite)
		global_PVs['TIFF1_FileName'].put(FileName)
	if int(variableDict['PostDarkImages']) > 0:
		print 'Capturing Post Dark Field'
		FileName = global_PVs['TIFF1_FileName'].get(as_string=True)
		global_PVs['TIFF1_FileName'].put(FileName+'_postdark')
		capture_multiple_projections(global_PVs, variableDict, int(variableDict['PreDarkImages']), FrameTypeDark)
		global_PVs['TIFF1_FileName'].put(FileName)
	close_shutters(global_PVs, variableDict)


	global_PVs['TIFF1_AutoSave'].put('No')
	global_PVs['TIFF1_Capture'].put(0)
	reset_CCD(global_PVs, variableDict)

def main():
	update_variable_dict(variableDict)
	start_scan()

if __name__ == '__main__':
	main()

