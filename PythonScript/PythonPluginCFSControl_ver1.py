# EnergyPlus, Copyright (c) 1996-2021, The Board of Trustees of the University
# of Illinois, The Regents of the University of California, through Lawrence
# Berkeley National Laboratory (subject to receipt of any required approvals
# from the U.S. Dept. of Energy), Oak Ridge National Laboratory, managed by UT-
# Battelle, Alliance for Sustainable Energy, LLC, and other contributors. All
# rights reserved.
#
# NOTICE: This Software was developed under funding from the U.S. Department of
# Energy and the U.S. Government consequently retains certain rights. As such,
# the U.S. Government has been granted for itself and others acting on its
# behalf a paid-up, nonexclusive, irrevocable, worldwide license in the
# Software to reproduce, distribute copies to the public, prepare derivative
# works, and perform publicly and display publicly, and to permit others to do
# so.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# (1) Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
# (2) Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
# (3) Neither the name of the University of California, Lawrence Berkeley
#     National Laboratory, the University of Illinois, U.S. Dept. of Energy nor
#     the names of its contributors may be used to endorse or promote products
#     derived from this software without specific prior written permission.
#
# (4) Use of EnergyPlus(TM) Name. If Licensee (i) distributes the software in
#     stand-alone form without changes from the version obtained under this
#     License, or (ii) Licensee makes a reference solely to the software
#     portion of its product, Licensee must refer to the software as
#     "EnergyPlus version X" software, where "X" is the version number Licensee
#     obtained under this License and may not use a different name for the
#     software. Except as specifically required in this Section (4), Licensee
#     shall not use in a company name, a product name, in advertising,
#     publicity, or other promotional activities any name, trade name,
#     trademark, logo, or other designation of "EnergyPlus", "E+", "e+" or
#     confusingly similar designation, without the U.S. Department of Energy's
#     prior written consent.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import math
import sys
print(sys.path)
print(sys.version)

import numpy as np
import random

## import pywincalc
import pywincalc


from pyenergyplus.plugin import EnergyPlusPlugin

def flatten_list(l):
    NRow= len(l)
    NCol = len(l[0])
    re = []
    for i in range(NRow):
        re = re+l[i]
    return re
def calculate_BSDF(EC_voltage,blind_height,bling_angle):
    # Another part to balabala

    optical_standard_path = r"D:\EP96\out\build\x64-Debug\Products\examples\standards\W5_NFRC_2003.std"
    optical_standard = pywincalc.load_standard(optical_standard_path)

    glazing_system_width = 4.0772  # width of the glazing system in meters
    glazing_system_height = 0.28435/2  # height of the glazing system in meters

    
    # Define the gap between the shade and the glazing
    gap_1 = pywincalc.Gap(pywincalc.PredefinedGasType.AIR, .0127)  # .0127 is gap thickness in meters

    bsdf_hemisphere = pywincalc.BSDFHemisphere.create(pywincalc.BSDFBasisType.FULL)

    # The BSDF data is currently stored as XML on igsdb.lbl.gov.  As a result it needs to be
    # parsed using the xml string parser instead of the json parser
    if bling_angle>=0:
        bsdf_shade = pywincalc.parse_bsdf_xml_file(r"D:\Users\fengf\Onedrive\Google Drive\TAMU\1_Project\NSF\2_Model\Daylighting Modeling\1_AutomatedSimulationWorkflow\Code\Tmx\blinds_Angle{}.xml".format(bling_angle))
    else:
        bsdf_shade = pywincalc.parse_bsdf_xml_file(r"D:\Users\fengf\Onedrive\Google Drive\TAMU\1_Project\NSF\2_Model\Daylighting Modeling\1_AutomatedSimulationWorkflow\Code\Tmx\blinds_Angle_N{}.xml".format(0-bling_angle))
    
    
    ## Generate glazing data file for Voltage to temp.dat
    geneGlazingFile(EC_voltage) 
    
    clear_3_path = r"D:\EP96\out\build\x64-Debug\Products\examples\GlazingData\temp.dat"
    clear_3 = pywincalc.parse_optics_file(clear_3_path)

    
    # Create a glazing system using the NFRC U environment in order to get NFRC U results
    # U and SHGC can be caculated for any given environment but in order to get results
    # The NFRC U and SHGC environments are provided as already constructed environments and Glazing_System
    # defaults to using the NFRC U environments
    glazing_system_u_environment = pywincalc.GlazingSystem(optical_standard=optical_standard,
                                                        solid_layers=[clear_3, 
                                                                        bsdf_shade],
                                                        gap_layers=[gap_1],
                                                        
                                                        
                                                        width_meters=glazing_system_width,
                                                        height_meters=glazing_system_height,
                                                        environment=pywincalc.nfrc_u_environments(),
                                                        bsdf_hemisphere=bsdf_hemisphere)


    glazing_system_shgc_environment = pywincalc.GlazingSystem(optical_standard=optical_standard,
                                                            solid_layers=[clear_3, bsdf_shade],
                                                            gap_layers=[gap_1],
                                                            width_meters=glazing_system_width,
                                                            height_meters=glazing_system_height,
                                                            environment=pywincalc.nfrc_shgc_environments(),
                                                            bsdf_hemisphere=bsdf_hemisphere)

    ## Absorptance matrix
    result_sol = glazing_system_u_environment.optical_method_results("SOLAR",0, 0) 
    fAbs_layer1 = result_sol.layer_results[0].front.absorptance.total_direct[2:2+145]
    bAbs_layer1 = result_sol.layer_results[0].back.absorptance.total_direct[2:2+145]
    
    fAbs_layer2 = result_sol.layer_results[1].front.absorptance.total_direct[2+145:2+290]
    bAbs_layer2 = result_sol.layer_results[1].back.absorptance.total_direct[2+145:2+290]
    

    ## Transmittance & Reflectance
    result_sys = result_sol.system_results
    TfSol = flatten_list(result_sys.front.transmittance.matrix)
    RbSol = flatten_list(result_sys.back.reflectance.matrix)
   

    # visible results
    result_vis = glazing_system_u_environment.optical_method_results("PHOTOPIC",0, 0)
    result_sys = result_vis.system_results
    
    TfVis = flatten_list(result_sys.front.transmittance.matrix)
    RbVis = flatten_list(result_sys.back.reflectance.matrix)

    print("calculated completed")
    return TfSol,RbSol,TfVis,RbVis,fAbs_layer1,bAbs_layer1,fAbs_layer2,bAbs_layer2

def calculate_BSDF_clear(EC_voltage,blind_height,bling_angle):
    # Another part to balabala

    optical_standard_path = r"D:\EP96\out\build\x64-Debug\Products\examples\standards\W5_NFRC_2003.std"
    optical_standard = pywincalc.load_standard(optical_standard_path)

    glazing_system_width = 4.0772  # width of the glazing system in meters
    glazing_system_height = 0.28435/2  # height of the glazing system in meters

    
    # Define the gap between the shade and the glazing
    gap_1 = pywincalc.Gap(pywincalc.PredefinedGasType.AIR, .0127)  # .0127 is gap thickness in meters

    bsdf_hemisphere = pywincalc.BSDFHemisphere.create(pywincalc.BSDFBasisType.FULL)


    ## Generate glazing data file for Voltage to temp.dat
    geneGlazingFile(EC_voltage) 
    
    clear_3_path = r"D:\EP96\out\build\x64-Debug\Products\examples\GlazingData\temp.dat"
    clear_3 = pywincalc.parse_optics_file(clear_3_path)

    
    # Create a glazing system using the NFRC U environment in order to get NFRC U results
    # U and SHGC can be caculated for any given environment but in order to get results
    # The NFRC U and SHGC environments are provided as already constructed environments and Glazing_System
    # defaults to using the NFRC U environments
    glazing_system_u_environment = pywincalc.GlazingSystem(optical_standard=optical_standard,
                                                        solid_layers=[clear_3],
                                                        width_meters=glazing_system_width,
                                                        height_meters=glazing_system_height,
                                                        environment=pywincalc.nfrc_u_environments(),
                                                        bsdf_hemisphere=bsdf_hemisphere)


    glazing_system_shgc_environment = pywincalc.GlazingSystem(optical_standard=optical_standard,
                                                            solid_layers=[clear_3],
                                                            width_meters=glazing_system_width,
                                                            height_meters=glazing_system_height,
                                                            environment=pywincalc.nfrc_shgc_environments(),
                                                            bsdf_hemisphere=bsdf_hemisphere)

    ## Absorptance matrix
    result_sol = glazing_system_u_environment.optical_method_results("SOLAR",0, 0) 
    fAbs_layer1 = result_sol.layer_results[0].front.absorptance.total_direct[1:1+145]
    bAbs_layer1 = result_sol.layer_results[0].back.absorptance.total_direct[1:1+145]
  

    ## Transmittance & Reflectance
    result_sys = result_sol.system_results
    TfSol = flatten_list(result_sys.front.transmittance.matrix)
    RbSol = flatten_list(result_sys.back.reflectance.matrix)
   

    # visible results
    result_vis = glazing_system_u_environment.optical_method_results("PHOTOPIC",0, 0)
    result_sys = result_vis.system_results
    
    TfVis = flatten_list(result_sys.front.transmittance.matrix)
    RbVis = flatten_list(result_sys.back.reflectance.matrix)

    print("calculated completed")
    return TfSol,RbSol,TfVis,RbVis,fAbs_layer1,bAbs_layer1

def read_Glazing_txt(fileName):
    ## read 
    re_Array = []
    with open(r"D:\EP96\out\build\x64-Debug\Products\examples\GlazingData"+"\\" + fileName) as fp:
        count  = 0
        for line in fp.readlines():
            count += 1
            if count >= 14:
                re_Array.append([float(x) for x in line.split('\t')])
    return re_Array

def geneGlazingFile(voltage_input):
    if voltage_input >4 or voltage_input <0:
        return 0
    else:
        fileName_Suffix = ["0o0","0o5","1o0","1o5","2o0","2o5","3o0","3o5","4o0"]

        voltage_low_idx = int(voltage_input/0.5)
        voltage_high_idx = int(voltage_input/0.5)+1
        
        voltage_low = voltage_low_idx * 0.5
        voltage_high = voltage_high_idx *0.5

        voltage_low_FileName = "aIGDB_NbTiO2_n{}V_v3.txt".format(fileName_Suffix[voltage_low_idx])
        voltage_high_FileName = "aIGDB_NbTiO2_n{}V_v3.txt".format(fileName_Suffix[voltage_high_idx])
        
        # read both upper and lower boundaries
        voltage_low_DF = read_Glazing_txt(voltage_low_FileName)
        voltage_high_DF = read_Glazing_txt(voltage_high_FileName)
        
        # create an empty list
        voltage_DF_new = []
        
        # interpolate
        for i in range(len(voltage_high_DF)):
            temp = [0]*4
            if voltage_input>voltage_low:
                temp[0] = voltage_high_DF[i][0]
                temp[1] = (voltage_high_DF[i][1]-voltage_low_DF[i][1])/0.5*(voltage_input-voltage_low)+voltage_low_DF[i][1]
                temp[2] = (voltage_high_DF[i][2]-voltage_low_DF[i][2])/0.5*(voltage_input-voltage_low)+voltage_low_DF[i][2]
                temp[3] = (voltage_high_DF[i][3]-voltage_low_DF[i][3])/0.5*(voltage_input-voltage_low)+voltage_low_DF[i][3]
            else:
                temp[0] = voltage_low_DF[i][0]
                temp[1] = voltage_low_DF[i][1]
                temp[2] = voltage_low_DF[i][2]
                temp[3] = voltage_low_DF[i][3]
            voltage_DF_new.append(temp)
        
        # add a header part
        lines = ["{ Units, Wavelength Units } SI Microns",
                "{ Thickness } 4",
                "{ Conductivity } 1",
                "{ IR Transmittance } TIR=0",
                "{ Emissivity, front back } Emis= 0.84 0.84",
                "{ }",
                "{ Product Name: NbTiO2_" + str(voltage_input)+"V }",
                "{ Manufacturer: Iowa State University }",
                "{ Type: Monolithic }",
                "{ Material: Glass }",
                "{ Appearance: Clear }",
                "{ NFRC ID: 3000003 }",
                "{ Acceptance: # }"]
        
        for i in range(len(voltage_DF_new)):
            lines.append("\t".join(["{0:.4f}".format(x) for x in voltage_DF_new[i]]))
        for i in range(len(lines)):
            lines[i] = lines[i]+"\n"
            
        # write data
        with open(r"D:\EP96\out\build\x64-Debug\Products\examples\GlazingData\temp.dat",'w+') as fp:
            fp.writelines(lines)
    return lines


class SetCFSState(EnergyPlusPlugin):
    ShadeStatusInteriorBlindOn = 6
    ShadeStatusOff = 0

    def __init__(self):
        super().__init__()
        self.handles_set = False
        self.handle_TestVariable_status = None  # only because actuators can be output vars ... yet
        self.CFS_Glz_Win1 = None
        self.CFS_Glz_Win2 = None
        self.CFS_Glz_Win3 = None
        self.CFS_Glz_Win4 = None
        self.CFS_Glz_Win5 = None
        self.CFS_Glz_Win6 = None
        self.CFS_Glz_Win7 = None
        self.CFS_Glz_Win8 = None
        self.CFS_Glz_Win9 = None
        self.CFS_Glz_Win10 = None
        self.CFS_Glz_Win1_clear = None
        self.CFS_Glz_Win2_clear = None
        self.CFS_Glz_Win3_clear = None
        self.CFS_Glz_Win4_clear = None
        self.CFS_Glz_Win5_clear = None
        self.CFS_Glz_Win6_clear = None
        self.CFS_Glz_Win7_clear = None
        self.CFS_Glz_Win8_clear = None
        self.CFS_Glz_Win9_clear = None
        self.CFS_Glz_Win10_clear = None

        self.Win1_Construct_handle = None
        self.Win2_Construct_handle = None
        self.Win2_Construct_handle = None
        self.Win3_Construct_handle = None
        self.Win4_Construct_handle = None
        self.Win5_Construct_handle = None
        self.Win6_Construct_handle = None
        self.Win7_Construct_handle = None
        self.Win8_Construct_handle = None
        self.Win9_Construct_handle = None
        self.Win10_Construct_handle = None
    
    def set_matrix_value(self,state,WinNum,TfSol_temp,RbSol_temp,TfVis_temp,RbVis_temp,fAbs_layer1_temp,bAbs_layer1_temp,fAbs_layer2_temp,bAbs_layer2_temp):
        
        #CFS_Glz_WinX_TfSol
        self.api.exchange.set_Plugin_Matrix(state, "CFS_Glz_Win{}_TfSol".format(WinNum),145,145,TfSol_temp)

        #CFS_Glz_WinX_RbSol
        self.api.exchange.set_Plugin_Matrix(state, "CFS_Glz_Win{}_RbSol".format(WinNum),145,145,RbSol_temp)

        #CFS_Glz_WinX_Tfvis
        self.api.exchange.set_Plugin_Matrix(state, "CFS_Glz_Win{}_Tfvis".format(WinNum),145,145,TfVis_temp)

        #CFS_Glz_WinX_Rbvis
        self.api.exchange.set_Plugin_Matrix(state, "CFS_Glz_Win{}_Rbvis".format(WinNum),145,145,RbVis_temp)

        #CFS_Glz_WinX_Layer_1_fAbs
        self.api.exchange.set_Plugin_Matrix(state, "CFS_Glz_Win{}_Layer_1_fAbs".format(WinNum),1,145,fAbs_layer1_temp)

        #CFS_Glz_WinX_Layer_1_bAbs
        self.api.exchange.set_Plugin_Matrix(state, "CFS_Glz_Win{}_Layer_1_bAbs".format(WinNum),1,145,bAbs_layer1_temp)

        #CFS_Glz_WinX_Layer_2_bAbs
        self.api.exchange.set_Plugin_Matrix(state, "CFS_Glz_Win{}_Layer_2_fAbs".format(WinNum),1,145,fAbs_layer2_temp)

        #CFS_Glz_WinX_Layer_2_fAbs
        self.api.exchange.set_Plugin_Matrix(state, "CFS_Glz_Win{}_Layer_2_bAbs".format(WinNum),1,145,bAbs_layer2_temp)

        return 0 
    def set_matrix_value_clear(self,state,WinNum,TfSol_temp,RbSol_temp,TfVis_temp,RbVis_temp,fAbs_layer1_temp,bAbs_layer1_temp):
        #CFS_Glz_WinX_TfSol
        self.api.exchange.set_Plugin_Matrix(state, "CFS_Glz_Win{}_TfSol".format(WinNum),145,145,TfSol_temp)

        #CFS_Glz_WinX_RbSol
        self.api.exchange.set_Plugin_Matrix(state, "CFS_Glz_Win{}_RbSol".format(WinNum),145,145,RbSol_temp)

        #CFS_Glz_WinX_Tfvis
        self.api.exchange.set_Plugin_Matrix(state, "CFS_Glz_Win{}_Tfvis".format(WinNum),145,145,TfVis_temp)

        #CFS_Glz_WinX_Rbvis
        self.api.exchange.set_Plugin_Matrix(state, "CFS_Glz_Win{}_Rbvis".format(WinNum),145,145,RbVis_temp)

        #CFS_Glz_WinX_Layer_1_fAbs
        self.api.exchange.set_Plugin_Matrix(state, "CFS_Glz_Win{}_Layer_1_fAbs".format(WinNum),1,145,fAbs_layer1_temp)

        #CFS_Glz_WinX_Layer_1_bAbs
        self.api.exchange.set_Plugin_Matrix(state, "CFS_Glz_Win{}_Layer_1_bAbs".format(WinNum),1,145,bAbs_layer1_temp)

        return 0 


    def on_begin_zone_timestep_before_init_heat_balance(self, state) -> int:  
        if not self.handles_set:    
            ###### Set construction using construction actuator
            ## 1) Get construction handle
            self.CFS_Glz_Win1 = self.api.exchange.get_construction_handle(state, "CFS_Glz_Win1")
            self.CFS_Glz_Win2 = self.api.exchange.get_construction_handle(state, "CFS_Glz_Win2")
            self.CFS_Glz_Win3 = self.api.exchange.get_construction_handle(state, "CFS_Glz_Win3")
            self.CFS_Glz_Win4 = self.api.exchange.get_construction_handle(state, "CFS_Glz_Win4")
            self.CFS_Glz_Win5 = self.api.exchange.get_construction_handle(state, "CFS_Glz_Win5")
            self.CFS_Glz_Win6 = self.api.exchange.get_construction_handle(state, "CFS_Glz_Win6")
            self.CFS_Glz_Win7 = self.api.exchange.get_construction_handle(state, "CFS_Glz_Win7")
            self.CFS_Glz_Win8 = self.api.exchange.get_construction_handle(state, "CFS_Glz_Win8")
            self.CFS_Glz_Win9 = self.api.exchange.get_construction_handle(state, "CFS_Glz_Win9")
            self.CFS_Glz_Win10 = self.api.exchange.get_construction_handle(state, "CFS_Glz_Win10")
            self.CFS_Glz_Win1_clear = self.api.exchange.get_construction_handle(state, "CFS_Glz_Win1_clear")
            self.CFS_Glz_Win2_clear = self.api.exchange.get_construction_handle(state, "CFS_Glz_Win2_clear")
            self.CFS_Glz_Win3_clear = self.api.exchange.get_construction_handle(state, "CFS_Glz_Win3_clear")
            self.CFS_Glz_Win4_clear = self.api.exchange.get_construction_handle(state, "CFS_Glz_Win4_clear")
            self.CFS_Glz_Win5_clear = self.api.exchange.get_construction_handle(state, "CFS_Glz_Win5_clear")
            self.CFS_Glz_Win6_clear = self.api.exchange.get_construction_handle(state, "CFS_Glz_Win6_clear")
            self.CFS_Glz_Win7_clear = self.api.exchange.get_construction_handle(state, "CFS_Glz_Win7_clear")
            self.CFS_Glz_Win8_clear = self.api.exchange.get_construction_handle(state, "CFS_Glz_Win8_clear")
            self.CFS_Glz_Win9_clear = self.api.exchange.get_construction_handle(state, "CFS_Glz_Win9_clear")
            self.CFS_Glz_Win10_clear = self.api.exchange.get_construction_handle(state, "CFS_Glz_Win10_clear")
            
            ## 2) Construction actuator
            self.Win1_Construct_handle = self.api.exchange.get_actuator_handle(state,
                                                                                "Surface",
                                                                                "Construction State",
                                                                                "28908_Wall_9_0_0_0_0_0_Win1")
            self.Win2_Construct_handle = self.api.exchange.get_actuator_handle(state,
                                                                                "Surface",
                                                                                "Construction State",
                                                                                "28908_Wall_9_0_0_0_0_0_Win2")
            self.Win3_Construct_handle = self.api.exchange.get_actuator_handle(state,
                                                                                "Surface",
                                                                                "Construction State",
                                                                                "28908_Wall_9_0_0_0_0_0_Win3")
            self.Win4_Construct_handle = self.api.exchange.get_actuator_handle(state,
                                                                                "Surface",
                                                                                "Construction State",
                                                                                "28908_Wall_9_0_0_0_0_0_Win4")
            self.Win5_Construct_handle = self.api.exchange.get_actuator_handle(state,
                                                                                "Surface",
                                                                                "Construction State",
                                                                                "28908_Wall_9_0_0_0_0_0_Win5")
            self.Win6_Construct_handle = self.api.exchange.get_actuator_handle(state,
                                                                                "Surface",
                                                                                "Construction State",
                                                                                "28908_Wall_9_0_0_0_0_0_Win6")  
            self.Win7_Construct_handle = self.api.exchange.get_actuator_handle(state,
                                                                                "Surface",
                                                                                "Construction State",
                                                                                "28908_Wall_9_0_0_0_0_0_Win7")
            self.Win8_Construct_handle = self.api.exchange.get_actuator_handle(state,
                                                                                "Surface",
                                                                                "Construction State",
                                                                                "28908_Wall_9_0_0_0_0_0_Win8")
            self.Win9_Construct_handle = self.api.exchange.get_actuator_handle(state,
                                                                                "Surface",
                                                                                "Construction State",
                                                                                "28908_Wall_9_0_0_0_0_0_Win9")
            self.Win10_Construct_handle = self.api.exchange.get_actuator_handle(state,
                                                                                "Surface",
                                                                                "Construction State",
                                                                                "28908_Wall_9_0_0_0_0_0_Win10")  
            self.handles_set = True                             


        ## This part implements the control strategy
        ## Step 0: print the currrent DayOfSim and TimeStepOfHour
        print("Day of Year: ", self.api.exchange.day_of_year(state),"   Hour:", self.api.exchange.hour(state))


        ## Step 1: Generate a random control signal.
        #EC_voltage = random.random()*4   # unique distributed [0,4]
        #blind_Angle = random.randint(-90,90)  # random integer between [-90,91)
        #blind_height = random.randint(0,10)   # random integer between [0,11)

        EC_voltage,blind_height,blind_Angle = 2.8013293941965576, 9,-41
        print(EC_voltage,blind_height,blind_Angle)
        TfSol,RbSol,TfVis,RbVis,fAbs_layer1,bAbs_layer1,fAbs_layer2,bAbs_layer2 =  calculate_BSDF(EC_voltage,blind_height,blind_Angle)
        TfSol_clear,RbSol_clear,TfVis_clear,RbVis_clear,fAbs_layer1_clear,bAbs_layer1_clear =  calculate_BSDF_clear(EC_voltage,blind_height,blind_Angle)
        
        with open("log.txt",'a+') as fp:
            fp.write("{},{},{}\n".format(EC_voltage,blind_height,blind_Angle))
        
        ## Window 1
        if blind_height >= 1:
            # Turn the shade of window 1 on
            self.api.exchange.set_actuator_value(state, self.Win1_Construct_handle, self.CFS_Glz_Win1)
            self.set_matrix_value(state,1,TfSol,RbSol,TfVis,RbVis,fAbs_layer1,bAbs_layer1,fAbs_layer2,bAbs_layer2)
            
        else:
            # Turn the shade of window 1 off
            self.api.exchange.set_actuator_value(state, self.Win1_Construct_handle, self.CFS_Glz_Win1_clear)
            self.set_matrix_value_clear(state,1,TfSol_clear,RbSol_clear,TfVis_clear,RbVis_clear,fAbs_layer1_clear,bAbs_layer1_clear)
        
        ## Window 2
        if blind_height >= 2:
            # Turn the shade of window 1 on
            self.api.exchange.set_actuator_value(state, self.Win2_Construct_handle, self.CFS_Glz_Win2)
            self.set_matrix_value(state,2,TfSol,RbSol,TfVis,RbVis,fAbs_layer1,bAbs_layer1,fAbs_layer2,bAbs_layer2)
        else:
            # Turn the shade of window 1 off
            self.api.exchange.set_actuator_value(state, self.Win2_Construct_handle, self.CFS_Glz_Win2_clear)
            self.set_matrix_value_clear(state,2,TfSol_clear,RbSol_clear,TfVis_clear,RbVis_clear,fAbs_layer1_clear,bAbs_layer1_clear)

        ## Window 3
        if blind_height >= 3:
            # Turn the shade of window 1 on
            self.api.exchange.set_actuator_value(state, self.Win3_Construct_handle, self.CFS_Glz_Win3)
            self.set_matrix_value(state,3,TfSol,RbSol,TfVis,RbVis,fAbs_layer1,bAbs_layer1,fAbs_layer2,bAbs_layer2)
        else:
            # Turn the shade of window 1 off
            self.api.exchange.set_actuator_value(state, self.Win3_Construct_handle, self.CFS_Glz_Win3_clear)
            self.set_matrix_value_clear(state,3,TfSol_clear,RbSol_clear,TfVis_clear,RbVis_clear,fAbs_layer1_clear,bAbs_layer1_clear)
        
        ## Window 4
        if blind_height >= 4:
            # Turn the shade of window 1 on
            self.api.exchange.set_actuator_value(state, self.Win4_Construct_handle, self.CFS_Glz_Win4)
            self.set_matrix_value(state,4,TfSol,RbSol,TfVis,RbVis,fAbs_layer1,bAbs_layer1,fAbs_layer2,bAbs_layer2)
        else:
            # Turn the shade of window 1 off
            self.api.exchange.set_actuator_value(state, self.Win4_Construct_handle, self.CFS_Glz_Win4_clear)
            self.set_matrix_value_clear(state,4,TfSol_clear,RbSol_clear,TfVis_clear,RbVis_clear,fAbs_layer1_clear,bAbs_layer1_clear)
        
        ## Window 5
        if blind_height >= 5:
            # Turn the shade of window 1 on
            self.api.exchange.set_actuator_value(state, self.Win5_Construct_handle, self.CFS_Glz_Win5)
            self.set_matrix_value(state,5,TfSol,RbSol,TfVis,RbVis,fAbs_layer1,bAbs_layer1,fAbs_layer2,bAbs_layer2)
        else:
            # Turn the shade of window 1 off
            self.api.exchange.set_actuator_value(state, self.Win5_Construct_handle, self.CFS_Glz_Win5_clear)
            self.set_matrix_value_clear(state,5,TfSol_clear,RbSol_clear,TfVis_clear,RbVis_clear,fAbs_layer1_clear,bAbs_layer1_clear)
        
        ## Window 6
        if blind_height >= 6:
            # Turn the shade of window 1 on
            self.api.exchange.set_actuator_value(state, self.Win6_Construct_handle, self.CFS_Glz_Win6)
            self.set_matrix_value(state,6,TfSol,RbSol,TfVis,RbVis,fAbs_layer1,bAbs_layer1,fAbs_layer2,bAbs_layer2)
        else:
            # Turn the shade of window 1 off
            self.api.exchange.set_actuator_value(state, self.Win6_Construct_handle, self.CFS_Glz_Win6_clear)
            self.set_matrix_value_clear(state,6,TfSol_clear,RbSol_clear,TfVis_clear,RbVis_clear,fAbs_layer1_clear,bAbs_layer1_clear)
        
        ## Window 7
        if blind_height >= 7:
            # Turn the shade of window 1 on
            self.api.exchange.set_actuator_value(state, self.Win7_Construct_handle, self.CFS_Glz_Win7)
            self.set_matrix_value(state,7,TfSol,RbSol,TfVis,RbVis,fAbs_layer1,bAbs_layer1,fAbs_layer2,bAbs_layer2)
        else:
            # Turn the shade of window 1 off
            self.api.exchange.set_actuator_value(state, self.Win7_Construct_handle, self.CFS_Glz_Win7_clear)
            self.set_matrix_value_clear(state,7,TfSol_clear,RbSol_clear,TfVis_clear,RbVis_clear,fAbs_layer1_clear,bAbs_layer1_clear)
        
        ## Window 8
        if blind_height >= 8:
            # Turn the shade of window 1 on
            self.api.exchange.set_actuator_value(state, self.Win8_Construct_handle, self.CFS_Glz_Win8)
            self.set_matrix_value(state,8,TfSol,RbSol,TfVis,RbVis,fAbs_layer1,bAbs_layer1,fAbs_layer2,bAbs_layer2)
        else:
            # Turn the shade of window 1 off
            self.api.exchange.set_actuator_value(state, self.Win8_Construct_handle, self.CFS_Glz_Win8_clear)
            self.set_matrix_value_clear(state,8,TfSol_clear,RbSol_clear,TfVis_clear,RbVis_clear,fAbs_layer1_clear,bAbs_layer1_clear)
        
        ## Window 9
        if blind_height >= 9:
            # Turn the shade of window 1 on
            print('bp')
            self.api.exchange.set_actuator_value(state, self.Win9_Construct_handle, self.CFS_Glz_Win9)
            self.set_matrix_value(state,9,TfSol,RbSol,TfVis,RbVis,fAbs_layer1,bAbs_layer1,fAbs_layer2,bAbs_layer2)
        else:
            # Turn the shade of window 1 off
            self.api.exchange.set_actuator_value(state, self.Win9_Construct_handle, self.CFS_Glz_Win9_clear)
            self.set_matrix_value_clear(state,9,TfSol_clear,RbSol_clear,TfVis_clear,RbVis_clear,fAbs_layer1_clear,bAbs_layer1_clear)
        
        ## Window 10
        if blind_height >= 10:
            # Turn the shade of window 1 on
            self.api.exchange.set_actuator_value(state, self.Win10_Construct_handle, self.CFS_Glz_Win10)
            self.set_matrix_value(state,10,TfSol,RbSol,TfVis,RbVis,fAbs_layer1,bAbs_layer1,fAbs_layer2,bAbs_layer2)
        else:
            # Turn the shade of window 1 off
            self.api.exchange.set_actuator_value(state, self.Win10_Construct_handle, self.CFS_Glz_Win10_clear)
            self.set_matrix_value_clear(state,10,TfSol_clear,RbSol_clear,TfVis_clear,RbVis_clear,fAbs_layer1_clear,bAbs_layer1_clear)

        return 0


    
