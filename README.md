# synthesizers
This respository holds Arduino sketches for Synthesizers

The files are:

ADF4351_W7GLF_TRANSVERTER 
  This file is for using either an ADF4350 or ADF4351 as the LO for a synthesizer.  It has an easily reconfigurable table of
  frequencies which can be selected from using a set of switches.  It runs on either an Arduino NANO or an Arduino UNO.

ADF4351_W7GLF_ver8
  This file is for using either an ADF4350 or ADF4351 for a frequency source.  It uses the DF Robot shield to enter the desired
  frequency.  It also has the ability to use discrete pushbuttons or a 4x4 matrix to act as the data entry device.
  This is an enhancement of work originally done by F1CJN.  It runs on either an Arduino NANO or an Arduino UNO.

ADF5355_W7GLF_06072023
  This file is for using either an ADF5355 for a frequency source.  It uses the DF Robot shield to enter the desired
  frequency.  It also has the ability to use discrete pushbuttons or a 4x4 matrix to act as the data entry device.
  This is an enhancement of work originally done by AA5C and F1CJN.  It allows setting frequency to the Hertz.
  This file will run on any of the following: Arduino NANO, Arduino UNO or Arduino DUE.

SI5351A_W7GLF_11212018
  This file is for using the SI5351 synthesizer.  It runs on either an Arduino NANO or Arduino UNO.
