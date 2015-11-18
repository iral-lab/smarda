'''
This file contains the basis for a color matching dummy db.
The utility of this file is to give a reference for viewed colors so that the colors viewed by the kinect can be matched to a medication.

author: Grace Chandler
date: 10-26-2015

'''
import sys 

#a function will be added here to map RGB values (in a range) to a color name.

#this dictionary will act as a representation for a 5 column table: Color, Medicine name, Brand name, Medicine class, mg, max daily dosage, amount taken(and in the real DB table ID). I have left out ID in this case beause it isn't particularly useful
colorMatch = dict()

#initialize the colors we already have
colorMatch["pink"] = ["Acetaminophen",["Tylenol", "Paracetamol" , "Panadol", "Mapap"], ["pain", "fever"],325,3250,0]
colorMatch["blue"] = ["Guaifenesin", ["Mucinex", "Tussin", "Robitussin Chest Congestion", "Humibid"], ["cough","cold"],300, 2400,0]
colorMatch["orange"] = ["Diazepam", ["Valium", "Diastat", "Diastat AcuDial", "Diazepam Intensol"],["generalized anxiety", "panic", "phobias"],5,60,0]


'''
    There will be more functionality added concerning drug interactions. For the sake of simplicity right now, we have chosen 3 very 
    separate medications. I double checked the interactions between these 3 drugs and they do not have any known interactions.
    
    An extension to this will be to use some purposefully conflicting drugs so that we can check for interactions

'''

def main():
    #input will be given as command line arguments
    color = sys.argv[1]
    print "color = "+ color
    
    #this is where RGB matching call will go
    
    #grab just the drug name
    drug = colorMatch[color][0]
    
    print "Drug = " + drug 
    print "Other names: " 
    for name in colorMatch[color][1]:
        sys.stdout.write(name + ",")
    print 
    print "Uses:"
    for use in colorMatch[color][2]:
        sys.stdout.write(use +  ",")
        
    #requesting medicine. Since we aren't using generated input, I arbitrarily chose the 'color' variable'
    medicine = color
    dose_request = 12
    
    #this variable keeps track of whether or not they have hit the closest dosage they can get to the maximum
    max_dosage = False
    while max_dosage != True and dose_request != 0:
        if colorMatch[medicine][-1]+ (colorMatch[medicine][-3]) < colorMatch[medicine][-2]:
            print("Dose Delivered")
            colorMatch[medicine][-1]+= (colorMatch[medicine][-3])
            dose_request -= 1
        else: 
            print("You have reached your maximum dosage. Please consult a physician if you are still experiencing unpleasant symptoms")
            max_dosage = True
main()