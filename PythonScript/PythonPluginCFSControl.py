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

    optical_standard_path = "./examples/standards/W5_NFRC_2003.std"
    optical_standard = pywincalc.load_standard(optical_standard_path)

    glazing_system_width = 1.0  # width of the glazing system in meters
    glazing_system_height = 1.0  # height of the glazing system in meters

    # Define the gap between the shade and the glazing
    gap_1 = pywincalc.Gap(pywincalc.PredefinedGasType.AIR, .0127)  # .0127 is gap thickness in meters

    bsdf_hemisphere = pywincalc.BSDFHemisphere.create(pywincalc.BSDFBasisType.FULL)

    # The BSDF data is currently stored as XML on igsdb.lbl.gov.  As a result it needs to be
    # parsed using the xml string parser instead of the json parser

    bsdf_shade = pywincalc.parse_bsdf_xml_file("./examples/products/2011-SA1.XML")

    clear_3_path = "./examples/products/CLEAR_3.DAT"
    clear_3 = pywincalc.parse_optics_file(clear_3_path)

    # Create a glazing system using the NFRC U environment in order to get NFRC U results
    # U and SHGC can be caculated for any given environment but in order to get results
    # The NFRC U and SHGC environments are provided as already constructed environments and Glazing_System
    # defaults to using the NFRC U environments
    glazing_system_u_environment = pywincalc.GlazingSystem(optical_standard=optical_standard,                                                       
                                                        ## From outside to inside
                                                        # Clear_3
                                                        # Gap(air)
                                                        # BSDF shade(Satine, white)
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

class SetCFSState(EnergyPlusPlugin):
    ShadeStatusInteriorBlindOn = 6
    ShadeStatusOff = 0

    def __init__(self):
        super().__init__()
        self.handles_set = False
        self.handle_TestVariable_status = None  # only because actuators can be output vars ... yet

    def on_begin_timestep_before_predictor(self, state) -> int:
        if not self.handles_set:
            self.handle_TestVariable_status = self.api.exchange.get_global_handle(
                state, "TestVariable_1"
            )
            self.handles_set = True
        self.api.exchange.set_global_value(state, self.handle_TestVariable_status, 1)
        
        ## 
        Control_action = True
        
        if (self.api.exchange.day_of_year(state)<=1) and (self.api.exchange.hour(state)<1):
            
            Control_action = True
        else:
            Control_action = False
        
        print("Day of Year: ", self.api.exchange.day_of_year(state),"   Hour:", self.api.exchange.hour(state))
        if Control_action:
            print("Take control action")
            TfSol,RbSol,TfVis,RbVis,fAbs_layer1,bAbs_layer1,fAbs_layer2,bAbs_layer2 =  calculate_BSDF(0,0,0)
            

            print('CFS_Glz_2_TfSol')
            #CFS_Glz_2_TfSol
            self.api.exchange.set_Plugin_Matrix(state, "CFS_Glz_2_TfSol",145,145,TfSol)

            #CFS_Glz_2_RbSol
            self.api.exchange.set_Plugin_Matrix(state, "CFS_Glz_2_RbSol",145,145,RbSol)

            #CFS_Glz_2_Tfvis
            self.api.exchange.set_Plugin_Matrix(state, "CFS_Glz_2_Tfvis",145,145,TfVis)

            #CFS_Glz_2_Rbvis
            self.api.exchange.set_Plugin_Matrix(state, "CFS_Glz_2_Rbvis",145,145,RbVis)

            #CFS_Glz_2_Layer_1_fAbs
            self.api.exchange.set_Plugin_Matrix(state, "CFS_Glz_2_Layer_1_fAbs",1,145,fAbs_layer1)

            #CFS_Glz_2_Layer_1_bAbs
            self.api.exchange.set_Plugin_Matrix(state, "CFS_Glz_2_Layer_1_bAbs",1,145,bAbs_layer1)

            #CFS_Glz_2_Layer_2_bAbs
            self.api.exchange.set_Plugin_Matrix(state, "CFS_Glz_2_Layer_2_fAbs",1,145,fAbs_layer2)

            #CFS_Glz_2_Layer_2_fAbs
            self.api.exchange.set_Plugin_Matrix(state, "CFS_Glz_2_Layer_2_bAbs",1,145,bAbs_layer2)
            
            #print('Hello world')

        return 0


    
