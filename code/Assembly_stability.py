import numpy as np
import math
from scipy.optimize import linprog

"""This program consist of three parts and is structured as follows:
    The first part: necessary functions (calculate contact force, find wrench cone, 
                    calculate wrench matrix, calculate stability using linear programming, print output)
    The second part: initialization of bodies and contacts for four cases: 1. default assemby collapsed,
                                                                           2. default assembly stands
                                                                           3. custom assembly collapsed
                                                                           4. custom assembly stands
    The third part: call functions to calculate wrench matrix, evaluate stability and print output.
"""
#class for body
class Body:
    def __init__(self, body_id, cm_x, cm_y, mass):
        self.body_id = body_id
        self.cm_x = cm_x
        self.cm_y = cm_y
        self.mass = mass

#class for contact
class Contact:
    def __init__(self, body_1, body_2, contact_x, contact_y, contact_normal, mu):
        self.body_1 = body_1
        self.body_2 = body_2
        self.contact_x = contact_x
        self.contact_y = contact_y
        self.contact_normal = contact_normal
        self.mu = mu


#-----------------------------The first part: necessary functions-------------------------

#function to calculate wrench based on contact force
def calc_F(x, y, normal): 
    f_x = round(math.cos(normal),4) #find force along x axis 
    f_y = round(math.sin(normal),4) #find force along y axis
    p = np.array([[x, y, 0]]) #constuct vector with contact point coordinates
    n = np.array([[f_x, f_y, 0]]) #find coordinates of a force normal
    m_z = np.cross(p, n) #find moment along Z axis
    F = (m_z, f_x, f_y) #find wrench
    return F

#function to calculate wrench cone of a contact
def find_wrench_cone(contact): 
    alpha_angle = math.atan(contact.mu) #find alpha angle based on friction coefficient mu
    f1 = calc_F(contact.contact_x, contact.contact_y, contact.contact_normal - alpha_angle) #first force (left side relative to a force normal) 
    f2 = calc_F(contact.contact_x, contact.contact_y, contact.contact_normal + alpha_angle) #first force (right side relative to a force normal) 
    return (f1, f2)

#function to determine a wrench position in resulting matrix column (where to put a wrench)
def body_rows(body_id):
    if body_id == 1:
        ind = (0,1,2) #rows 0, 1, 2 of a column for the first body
    elif body_id == 2:
        ind = (3,4,5) #rows 3, 4, 5 of a column for the second body
    elif body_id == 3:
        ind = (6,7,8) #rows 6, 7, 8 of a column for the third body (if exist)
    elif body_id == 4:
        ind = (9,10,11) #rows 9, 10, 11 of a column for the fourth body (if exist)
    elif body_id == 5:
        ind = (12,13,14) #rows 12, 13, 14 of a column for the fifth body (if exist)
    return ind

#function to calculate a system of equations in a form of resulting wrench matrix
def calculate_wrench_matrix(bodies_list, contacts_list):
    #calculate size of resulting wrenches matrix (system of equations) based on number of bodies and contacts
    wrenches_matrix_size = np.zeros((3 * len(bodies_list), 2 * len(contacts_list) + len(bodies_list)))
    rows = wrenches_matrix_size.shape[0] #total number of rows
    columns = wrenches_matrix_size.shape[1] #total number of columns
    wrenches_matrix = np.zeros((rows,1)) #create first column of the resulting wrench matrix and fill in by zeros (zeros will be replaced by actual wrenches later)

    #calculating and filling in the first two columns of the resulting wrenches matrix (first wrench cone)
    if contacts_list[0].body_2 == 0: #if body 2 is ground, then calculate only one wrench cone corresponding to the body 1 
        contact_wrench = find_wrench_cone(contacts_list[0]) #find wrench cone
        #change the first column from zeros to contact wrench we found above
        np.put(wrenches_matrix, [body_rows(contacts_list[0].body_1)[0], body_rows(contacts_list[0].body_1)[1], body_rows(contacts_list[0].body_1)[2]], [contact_wrench[0][1], contact_wrench[0][2], contact_wrench[0][0][0][2]])
        temp_column = np.zeros((rows, 1)) #create one more column with zeros (it will be the second column of the resulting wrench matrix later)
        #change zeros to actual wrench
        np.put(temp_column, [body_rows(contacts_list[0].body_1)[0], body_rows(contacts_list[0].body_1)[1], body_rows(contacts_list[0].body_1)[2]], [contact_wrench[1][1], contact_wrench[1][2], contact_wrench[1][0][0][2]])
        wrenches_matrix = np.append(wrenches_matrix, temp_column, axis=1) #append column to the resulting wrenches matrix 
    elif contacts_list[0].body_1 == 0: #if body 1 is ground, then calculate only one wrench cone corresponding to the body 2. Actions are the same as above
        contact_wrench = find_wrench_cone(contacts_list[0]) #find wrench cone
        #insert the first wrench to the first column
        np.put(wrenches_matrix, [body_rows(contacts_list[0].body_2)[0], body_rows(contacts_list[0].body_2)[1], body_rows(contacts_list[0].body_2)[2]], [contact_wrench[0][1], contact_wrench[0][2], contact_wrench[0][0][0][2]])
        temp_column = np.zeros((rows, 1)) #create second column and fill in with zeros
        #change zeros to actual wrench
        np.put(temp_column, [body_rows(contacts_list[0].body_2)[0], body_rows(contacts_list[0].body_2)[1], body_rows(contacts_list[0].body_2)[2]], [contact_wrench[1][1], contact_wrench[1][2], contact_wrench[1][0][0][2]])
        wrenches_matrix = np.append(wrenches_matrix, temp_column, axis=1) #append column to the resulting wrenches matrix 
    else: #if body 1 and body 2 are not ground, calculate two wrech cones for each body
        contact_wrench = find_wrench_cone(contacts_list[0]) #find wrench cone
        #insert the first wrench for both bodies to a first column
        np.put(wrenches_matrix, [body_rows(contacts_list[0].body_1)[0], body_rows(contacts_list[0].body_1)[1], body_rows(contacts_list[0].body_1)[2]], [contact_wrench[0][1], contact_wrench[0][2], contact_wrench[0][0][0][2]])
        np.put(wrenches_matrix, [body_rows(contacts_list[0].body_2)[0], body_rows(contacts_list[0].body_2)[1], body_rows(contacts_list[0].body_2)[2]], [-contact_wrench[0][1], -contact_wrench[0][2], -contact_wrench[0][0][0][2]])    
        temp_column = np.zeros((rows, 1)) #create a column with zeros to replace it with second wrench
        #insert the second wrench for both bodies to a second column
        np.put(temp_column, [body_rows(contacts_list[0].body_1)[0], body_rows(contacts_list[0].body_1)[1], body_rows(contacts_list[0].body_1)[2]], [contact_wrench[1][1], contact_wrench[1][2], contact_wrench[1][0][0][2]])
        np.put(temp_column, [body_rows(contacts_list[0].body_2)[0], body_rows(contacts_list[0].body_2)[1], body_rows(contacts_list[0].body_2)[2]], [-contact_wrench[1][1], -contact_wrench[1][2], -contact_wrench[1][0][0][2]])
        wrenches_matrix = np.append(wrenches_matrix, temp_column, axis=1) #append column to the resulting wrenches matrix 

    #calculating and filling the rest columns of the resulting wrenches matrix
    for contact in contacts_list[1:]: #for each contact (except the first one) find wrench cone
        contact_wrench = find_wrench_cone(contact) #find wrench cone
        if contact.body_2 == 0: #if body 2 is ground, then calculate only one wrench cone corresponding to the body 1 
            temp_column = np.zeros((rows, 1)) #create a column with zeros to replace it with first wrench
            #insert the first wrench for the first body to a first column
            np.put(temp_column, [body_rows(contact.body_1)[0], body_rows(contact.body_1)[1], body_rows(contact.body_1)[2]], [contact_wrench[0][1], contact_wrench[0][2], contact_wrench[0][0][0][2]])
            wrenches_matrix = np.append(wrenches_matrix, temp_column, axis=1) #append column to the resulting wrenches matrix 
            temp_column = np.zeros((rows, 1))#create a column with zeros to replace it with second wrench
            #insert the second wrench for the first body to a second column
            np.put(temp_column, [body_rows(contact.body_1)[0], body_rows(contact.body_1)[1], body_rows(contact.body_1)[2]], [contact_wrench[1][1], contact_wrench[1][2], contact_wrench[1][0][0][2]])
            wrenches_matrix = np.append(wrenches_matrix, temp_column, axis=1) #append column to the resulting wrenches matrix 
        elif contact.body_1 == 0: #if body 1 is ground, then calculate only one wrench cone corresponding to the body 2 
            temp_column = np.zeros((rows, 1))#create a column with zeros to replace it with first wrench
            #insert the first wrench for the second body to a first column
            np.put(temp_column, [body_rows(contact.body_2)[0], body_rows(contact.body_2)[1], body_rows(contact.body_2)[2]], [contact_wrench[0][1], contact_wrench[0][2], contact_wrench[0][0][0][2]])
            wrenches_matrix = np.append(wrenches_matrix, temp_column, axis=1) #append column to the resulting wrenches matrix 
            temp_column = np.zeros((rows, 1)) #create a column with zeros to replace it with second wrench
            #insert the second wrench for the second body to a second column
            np.put(temp_column, [body_rows(contact.body_2)[0], body_rows(contact.body_2)[1], body_rows(contact.body_2)[2]], [contact_wrench[1][1], contact_wrench[1][2], contact_wrench[1][0][0][2]])
            wrenches_matrix = np.append(wrenches_matrix, temp_column, axis=1) #append column to the resulting wrenches matrix 
        else: #if body 1 and body 2 are not ground, calculate two wrech cones for each body
            temp_column = np.zeros((rows, 1)) #create a column with zeros to replace it with first wrench
            #insert the first wrench for both bodies to a first column
            np.put(temp_column, [body_rows(contact.body_1)[0], body_rows(contact.body_1)[1], body_rows(contact.body_1)[2]], [contact_wrench[0][1], contact_wrench[0][2], contact_wrench[0][0][0][2]])
            np.put(temp_column, [body_rows(contact.body_2)[0], body_rows(contact.body_2)[1], body_rows(contact.body_2)[2]], [-contact_wrench[0][1], -contact_wrench[0][2], -contact_wrench[0][0][0][2]])
            wrenches_matrix = np.append(wrenches_matrix, temp_column, axis=1) #append column to the resulting wrenches matrix 
            temp = np.zeros((rows, 1)) #create a column with zeros to replace it with second wrench
            #insert the second wrench for both bodies to a second column
            np.put(temp_column, [body_rows(contact.body_1)[0], body_rows(contact.body_1)[1], body_rows(contact.body_1)[2]], [contact_wrench[1][1], contact_wrench[1][2], contact_wrench[1][0][0][2]])
            np.put(temp_column, [body_rows(contact.body_2)[0], body_rows(contact.body_2)[1], body_rows(contact.body_2)[2]], [-contact_wrench[1][1], -contact_wrench[1][2], -contact_wrench[1][0][0][2]])
            wrenches_matrix = np.append(wrenches_matrix, temp_column, axis=1) #append column to the resulting wrenches matrix 

    #calculating gravitational wrenches
    for body in bodies_list: #for each body in the list
        f_ext = - body.mass * 9.81 #find external force
        m_ext = np.cross((body.cm_x, body.cm_y), (0, f_ext)) #find external moment
        wrench_ext = (0, f_ext, m_ext.item(0)) #find external wrench
        temp = np.zeros((rows, 1)) #create a column with zeros to replace it with actual wrench
        np.put(temp, [body_rows(body.body_id)[0:3]], [wrench_ext[0:3]]) #insert the wrench instead of zeros
        wrenches_matrix = np.append(wrenches_matrix, temp, axis=1) #append column to the resulting wrenches matrix 
    return wrenches_matrix, rows, columns


#function to calculate stability of an assembly using linear programming
def calculate_stability(wrenches_matrix, rows, columns, bodies_list):
    c = np.ones(columns) #coefficients to be minimized. Initially set as ones 
    Aeq = wrenches_matrix #equality constraint matrix
    beq = np.zeros(rows) #equality constraint vector
    #calculating bounds that are representing non-negative coefficients for internal wrenches and ones for external (gravity) wrenches. According to the excercise in the book
    top_bound = ([None] * (columns - len(bodies_list))) #set none as top bound for internal wrenches
    for i in bodies_list: #set ones as top bound for external wrenches (gravity)
        top_bound.append(1)
    top_bounds = np.array([top_bound])
    bottom_bound = ([0] * (columns - len(bodies_list))) #set 0 as bottom bound for internal wrenches
    for i in bodies_list:
        bottom_bound.append(1)
    bottom_bounds = np.array([bottom_bound])
    total_bounds = [] #initialize resulting list for bounds
    for i in range(columns): #filling in resulting bounds list
        total_bounds.append((bottom_bounds[0][i], top_bounds[0][i]))
    res = linprog(c, A_eq=Aeq, b_eq=beq, bounds=total_bounds, options={'tol': .000001}) #call of the linprog function
    return res

#function to print output for all the cases
def print_and_save_output(result_case, res):
    print('Bodies in this case are:')
    print('Body id, center of mass x, center of mass y, mass')
    #print bodies parameters 
    if result_case == 1:
        for i in bodies_list_case1:
            print(i.body_id, i.cm_x, i.cm_y, i.mass)
    elif result_case == 2:
        for i in bodies_list_case2:
            print(i.body_id, i.cm_x, i.cm_y, i.mass)
    elif result_case == 3:
        for i in bodies_list_case3:
            print(i.body_id, i.cm_x, i.cm_y, i.mass)
    elif result_case == 4:
        for i in bodies_list_case4:
            print(i.body_id, i.cm_x, i.cm_y, i.mass)
    print('Contacts in this case are:')
    print('Body id1, body id2, x, y, normal, mu')
    #print contacts parameters
    if result_case == 1:
        for i in contacts_list_case1:
            print(i.body_1, i.body_2, i.contact_x, i.contact_y, i.contact_normal, i.mu)
    elif result_case == 2:
        for i in contacts_list_case2:
            print(i.body_1, i.body_2, i.contact_x, i.contact_y, i.contact_normal, i.mu)
    elif result_case == 3:
        for i in contacts_list_case3:
            print(i.body_1, i.body_2, i.contact_x, i.contact_y, i.contact_normal, i.mu)
    elif result_case == 4:
        for i in contacts_list_case4:
            print(i.body_1, i.body_2, i.contact_x, i.contact_y, i.contact_normal, i.mu)
    if res.success == True:
        print('The assembly stands')
        print('Coefficients: ', np.round(res.x, 3))
    else:
        print('The assembly collapsed')
    print('\n')


#----------------The second part: initialization of bodies and contacts for four cases-------------------------

#Case #1: Standard assembly (two bodies) is collapsing
body_1 = Body(1, 25, 35, 2) #(id, center of mass x, center of mass y, mass)
body_2 = Body(2, 66, 42, 5)
bodies_list_case1 = [body_1, body_2]

contact1 = Contact(1, 0, 0, 0, math.pi/2, 0.1) #(body id1, body id2, x, y, normal, mu)
contact2 = Contact(1, 2, 60, 60, math.pi, 0.5)
contact3 = Contact(2, 0, 60, 0, math.pi/2, 0.5)
contact4 = Contact(2, 0, 72, 0, math.pi/2, 0.5)
contacts_list_case1 = [contact1, contact2, contact3, contact4]

#Case #2: Standard assembly (two bodies) is standing
body_1 = Body(1, 25, 35, 2) #(id, center of mass x, center of mass y, mass)
body_2 = Body(2, 66, 42, 10)
bodies_list_case2 = [body_1, body_2]

contact1 = Contact(1, 0, 0, 0, math.pi/2, 0.5) #(body id1, body id2, x, y, normal, mu)
contact2 = Contact(1, 2, 60, 60, math.pi, 0.5)
contact3 = Contact(2, 0, 60, 0, math.pi/2, 0.5)
contact4 = Contact(2, 0, 72, 0, math.pi/2, 0.5)
contacts_list_case2 = [contact1, contact2, contact3, contact4]

#Case #3: Custom assembly (three bodies) is collapsing
body_1 = Body(1, 25, 35, 2) #(id, center of mass x, center of mass y, mass)
body_2 = Body(2, 66, 42, 5)
body_3 = Body(3, 66, 90, 5)
bodies_list_case3 = [body_1, body_2, body_3]

contact1 = Contact(1, 0, 0, 0, math.pi/2, 0.4) #(body id1, body id2, x, y, normal, mu)
contact2 = Contact(1, 2, 60, 60, math.pi, 0.5)
contact3 = Contact(2, 0, 60, 0, math.pi/2, 0.5)
contact4 = Contact(2, 0, 72, 0, math.pi/2, 0.5)
contact5 = Contact(2, 3, 60, 84, -math.pi/2, 0.5)
contact6 = Contact(2, 3, 72, 84, -math.pi/2, 0.5)
contacts_list_case3 = [contact1, contact2, contact3, contact4, contact5, contact6]

#Case #4: Custom assembly (three bodies) is standing
body_1 = Body(1, 25, 35, 2) #(id, center of mass x, center of mass y, mass)
body_2 = Body(2, 66, 42, 5)
body_3 = Body(3, 66, 90, 5)
bodies_list_case4 = [body_1, body_2, body_3]

contact1 = Contact(1, 0, 0, 0, math.pi/2, 0.5) #difference is only here in friction coefficient mu
contact2 = Contact(1, 2, 60, 60, math.pi, 0.5) #(body id1, body id2, x, y, normal, mu)
contact3 = Contact(2, 0, 60, 0, math.pi/2, 0.5)
contact4 = Contact(2, 0, 72, 0, math.pi/2, 0.5)
contact5 = Contact(2, 3, 60, 84, -math.pi/2, 0.5)
contact6 = Contact(2, 3, 72, 84, -math.pi/2, 0.5)
contacts_list_case4 = [contact1, contact2, contact3, contact4, contact5, contact6]


#----------------The third part: call functions to calculate wrench matrix, evaluate stability and print output-------------------------
#for each cases we find resulting wrenches matrix, then apply linear programming to determine stability. Then print output

#case 1:
print("Case 1")
wrenches_case1 = calculate_wrench_matrix(bodies_list_case1, contacts_list_case1)
result_case1 = calculate_stability(wrenches_case1[0], wrenches_case1[1], wrenches_case1[2], bodies_list_case1)
print_and_save_output(1, result_case1)
    
#case 2:
print("Case 2")
wrenches_case2 = calculate_wrench_matrix(bodies_list_case2, contacts_list_case2)
result_case2 = calculate_stability(wrenches_case2[0], wrenches_case2[1], wrenches_case2[2], bodies_list_case2)
print_and_save_output(2, result_case2)

#case 3:
print("Case 3")
wrenches_case3 = calculate_wrench_matrix(bodies_list_case3, contacts_list_case3)
result_case3 = calculate_stability(wrenches_case3[0], wrenches_case3[1], wrenches_case3[2], bodies_list_case3)
print_and_save_output(3, result_case3)

#case 4:
print("Case 4")
wrenches_case4 = calculate_wrench_matrix(bodies_list_case4, contacts_list_case4)
result_case4 = calculate_stability(wrenches_case4[0], wrenches_case4[1], wrenches_case4[2], bodies_list_case4)
print_and_save_output(4, result_case4)
print(wrenches_case4)