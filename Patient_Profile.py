'''
Created on Oct 12, 2015

@author: gracekathrynchandler

This file hashes out the basics of a patient profile to be used for the patient profile database for the MARDA system.
Allows for the creation of a basic patient object with standard traits
Includes a section for medically specific data
Includes a section for recording vitals and averaging vitals
'''

class Patient_Profile:
 

    def __init__(self, name, age, height, weight, zipc):
        self.name = name
        self.age = age
        self.height = height
        self.weight = weight
        self.zip = zipc
    
    def set_name(self,name):
        self.name = name
        
    def set_age(self,age):
        self.age = age
        
    def set_height(self,height):
        self.height=height
        
    def set_weight(self,weight):
        self.weight = weight
    
    def set_zip(self,zipc):
        self.zip = zipc
        
    def get_name(self):
        return self.name
    
    def get_age(self):
        return self.age
    
    def get_height(self):
        return self.height
    
    def get_weight(self):
        return self.weight
    
    def get_zip(self):
        return self.zip
    
    '''
        The next section covers more medically specific information such as primary care physician, medications,current
        treatments, allergies and scheduling information.
        
        Additional functionality will be added to check for cross-interactions between drugs
    
    '''
    
    #ABO- A,B,AB,O; rhesus- positive or negative
    def set_blood_type(self,ABO,rhesus):
        self.ABO = ABO
        self.rhesus = rhesus
        
    #PCP = primary care physician
    def set_pcp(self,pcp):
        self.pcp = pcp
        
    def set_medications(self,meds_list):
        self.meds_list = meds_list
        
    def set_treatments(self,treatments):
        self.treatments = treatments
    
    def set_allergies(self,allergies):
        self.allergies = allergies
        
    def get_abo(self):
        return self.ABO
    
    def get_rhesus(self):
        return self.rhesus
    
    #returns blood type as one string, ie "A-Positive"
    def get_full_blood_type(self):
        full_blood_type = self.ABO+ " " + self.rhesus
        return full_blood_type
    
    def get_pcp(self):
        return self.pcp
    
    def get_medications(self):
        return self.meds_list
    
    def get_treatments(self):
        return self.treatments
    
    def get_allergies(self):
        return self.allergies
    
    def add_med(self,med):
        self.meds_list.append(med)

    def add_treatment(self,treatment):
        self.treatments.append(treatment)
        
    def add_allergy(self,allergy):
        self.allergies.append(allergy)
        
        
    '''
    Variable Vitals
    
    This section of the class will cover variable traits such as vitals
    
    Any time-related changes will also be monitored here
    '''
    #format of vitals {blood_pressure:[bp,time],[bp,time],heart_rate:[hr,time}.....}
    def init_vitals(self):
        self.vitals = dict()
        self.vitals["blood_pressure"] = []
        self.vitals["heart_rate"] = []
        self.vitals["respiratory_rate"] =[]
        self.vitals["body_temperature"] = []
        
    def record_blood_pressure(self,bp,time):
        self.vitals["blood_pressure"] = [bp,time]
        
    def record_body_temperature(self,body_temp,time):
        self.vitals["body_temp"] = [body_temp,time]
        
    def record_heart_rate(self,hr,time):
        self.vitals["heart_rate"] = [hr,time]
        
    def record_respiratory_rate(self,resp_rate,time):
        self.vitals["respiratory_rate"] = [resp_rate,time]
        
    #Eventually make this better to account for the time the vitals were taken. 
    def average_vital(self,vital):
        total = 0
        for event in self.vitals[vital]:
            total += event[0]
        average = float(total)/float(len(self.vitals[vital]))
        return average
    
    #Eventually make this better to account for the time the vitals were taken. 
    def average_all_vitals(self):
        averages = dict()
        for vital in self.vitals.keys():
            averages[vital] = self.average_vital(vital)
            
        self.average_vitals = averages
        
        