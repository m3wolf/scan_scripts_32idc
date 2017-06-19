from epics import PV
import time

#DAC_array = [5.0, 5.05, 5.08, 5.1, 5.12, 5.14, 8.54, 8.62, 8.66, 8.7, 8.75, 8.78]
DAC_array = [5.0, 5.05, 5.08, 5.1, 5.12, 8.54, 8.62, 8.66, 8.7, 8.75]
rot_diamond = PV('32idcTXM:DAC1_3.VAL')

#time.sleep(2700)

for iLoop in DAC_array:
    print('*** Tomo forDAC = ' + str(iLoop))
    rot_diamond.put(iLoop)
    execfile('/home/beams/USR32IDC/TXM/scanScript/ag_py/tomo_step_scan.py')
    
    