'''
Created on Oct 13, 2015

@author: gracekathrynchandler

This file provides the functionality of creating a database of patient profile objects for a single instance of the MARDA robot.
'''

import Patient_Profile

class Patient_DB:
    
    def __init__(self, MARDA_ID,administrator_physician):
        self.marda_id = MARDA_ID
        self.admin_physician = administrator_physician
        self.patients = dict()
        self.physician_users = []
        
    #this will need to like have some real security added at some point, people have to be held accountable
    def set_admin_physician(self,administrator_physician):
        self.admin_physician = administrator_physician
        
    def add_patient(self,patient):
        self.patients[patient.get_name()] = patient
        
    '''
        At some point we need to add physician profiles because different medical professionals can adminster different drugs
        The resident physician/nurse in control of the MARDA unit at the time is responsible for the care of the patient
        Tele-monitoring by physicians/nurses will be a safety net for patients/allow the MARDA to contact someone in case of emergency
        or other question.
    
    '''
    def add_physician_user(self,physician_user):
        self.physician_user.append(physician_user)
        
    def get_marda_id(self):
        return self.marda_id
    
    def get_admin_physician(self):
        return self.admin_physician
    
    def get_patients(self):
        return self.patients

    def get_physician_users(self):
        return self.physician_users
    
    def find_patient(self,patient_name):
        for patient in self.patients.keys():
            if patient == patient_name:
                return 1,self.patients[patient]
        return 0,None
    
    
    
        
    
        