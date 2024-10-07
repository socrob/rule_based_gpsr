#! /usr/bin/env python3
import re
import rospy
import copy

CONFIRM_TIME = 30
CONFIRM_TRIES = 2

POUR_PLACE = "kitchen table"
TAKEOUT_PLACE = "counter"
TAKEOUT_FIND_GARBAGE = "kitchen"
TAKEOUT_OBJ_NAME = "handbag"

ASK_LEAVE_LOC = "exit"
OPERATOR_LOC = "{instruction_point}"

# Category for the food and drinks in the serve task
SERVE_EAT="snacks"
SERVE_DRINK="drinks"

#######################################
###!    GENERAL PURPOSE FUNCTS     !###
#######################################

def say_await_reply(rule_based_planner, sentence="", say_first=True, time_secs=CONFIRM_TIME, tries=CONFIRM_TRIES, basic_reply=True, use_keyword_detection=False, w_nlu_result=False):
    county = 0
    skip = False
    reply = None
    nlu_result = None

    if sentence == "":
        skip = True
    
    while county<tries:
        if not(skip) and say_first:
            rospy.logwarn(sentence)
            rule_based_planner.execution_order('speak',voice_command=sentence)

        if not(w_nlu_result):
            reply = rule_based_planner.tiago_api.speech.wait_for_reply(time_secs, basic_reply=basic_reply)
        else:
            reply, nlu_result = rule_based_planner.tiago_api.speech.wait_for_reply(time_secs, basic_reply=basic_reply, use_keyword_detection=use_keyword_detection, w_nlu_result=w_nlu_result)

        if (reply != None and reply != []) or rospy.is_shutdown():
            break

        if not(skip) and not(say_first) and (county < tries-1):
            rospy.logwarn(sentence)
            rule_based_planner.execution_order('speak',voice_command=sentence)
        
        county += 1

    if not(w_nlu_result):
        return reply
    else:
        return reply, nlu_result


#######################################
###!        TASKS FUNCTS           !###
#######################################
def convert_task_to_actions(rule_based_planner, task):
    if(task.verb == 'take'):
        pick_task(rule_based_planner, task)
        rule_based_planner.picked_value = True
        rule_based_planner.manip_flag = True

    elif(task.verb == 'place'):
        place_task(rule_based_planner, task, rule_based_planner.picked_value)
        rule_based_planner.picked_value = False
        rule_based_planner.manip_flag = True

    elif(task.verb == 'bring'):
        bring_task(rule_based_planner, task)
        rule_based_planner.manip_flag = True

    elif(task.verb == 'deliver'):
        deliver_task(rule_based_planner, task, rule_based_planner.picked_value)
        rule_based_planner.picked_value = False
        rule_based_planner.manip_flag = True

    elif(task.verb == 'speak'):
        speak_task(rule_based_planner, task)

    elif(task.verb == 'gopl'):
        move_task(rule_based_planner, task)
        
    elif(task.verb == 'find'):
        find_task(rule_based_planner, task)

    elif(task.verb == 'guide'):
        guide_task(rule_based_planner, task)

    elif(task.verb == 'meet'):
        meet_task(rule_based_planner, task)

    elif(task.verb == 'greet'):
        greet_task(rule_based_planner, task)

    elif(task.verb == 'introduce'):
        introduce_task(rule_based_planner, task)

    elif(task.verb == 'tellcount'):
        tellcount_task(rule_based_planner, task)

    elif(task.verb == 'tellmeet'):
        tellmeet_task(rule_based_planner, task)

    elif(task.verb == 'tellobject'):
        tellobject_task(rule_based_planner, task)

    elif(task.verb == 'follow'):
        follow_task(rule_based_planner, task)

    elif(task.verb == 'takeout'):
        takeout_task(rule_based_planner, task)

    elif(task.verb == 'askleave'):
        askleave_task(rule_based_planner, task)
        
    elif(task.verb == 'opendoor'):
        opendoor_task(rule_based_planner, task)
        
    elif(task.verb == 'closedoor'):
        closedoor_task(rule_based_planner, task)

    elif(task.verb == 'place_in'):
        place_in_task(rule_based_planner, task, rule_based_planner.picked_value)
        rule_based_planner.picked_value = False
        rule_based_planner.manip_flag = True

    elif(task.verb == 'pour'):
        pour_task(rule_based_planner, task)
        rule_based_planner.picked_value = False
        rule_based_planner.manip_flag = True

    elif(task.verb == 'telldescribe'):
        telldescribe(rule_based_planner, task)

    elif(task.verb == 'serve'):
        serve_task(rule_based_planner, task)

    elif(task.verb == 'cleanup'):
        #rule_based_planner.execution_order('speak',voice_command="Cleanup tasks are not implemented at this stage.")
        #cleanup_task(rule_based_planner, task)
        pass

    elif(task.verb == 'end_speech'):
        if not(finito_flag):
            rule_based_planner.plan_termination()
            finito_flag = True

    else:
        print("-" + task.verb + "-")
        rospy.logwarn('Warning!!!: Action doesn\'t exist!!!')

def pick_task(rule_based_planner, task):
    if not rule_based_planner.has_found_object:
        rule_based_planner.execution_order('find_object', 
                                           object=task.object1, object_info=task.object2,
                                           source=task.source, output_variable="found_object")
        if rule_based_planner.action_success:
            rule_based_planner.has_found_object = True
        else:
            rule_based_planner.has_found_object = False

    rule_based_planner.execution_order('pick', object="{found_object}")


def place_task(rule_based_planner, task, pick_status):
    if not(pick_status):
        pick_task(rule_based_planner, task)

    rule_based_planner.execution_order('place', destination=task.destination, object="{found_object}")

    # Assume end of interaction with object
    rule_based_planner.has_found_object = False


def bring_task(rule_based_planner, task): #! person1: "operator" + always pick missing
    deliver_task(rule_based_planner, task, pick_status=False)


def deliver_task(rule_based_planner, task, pick_status):
    if not(pick_status):
       pick_task(rule_based_planner, task)
    
    if task.person1 == "operator":
        rule_based_planner.execution_order('move', destination=OPERATOR_LOC)
    else:
        rule_based_planner.execution_order('find_person', person=task.person1, source=task.destination, person_info = task.person2)

    rule_based_planner.execution_order('deliver', object="{found_object}", person=task.person1)


def speak_task(rule_based_planner, task):
    # If no person was found in previous action, find a person to speak to
    if not rule_based_planner.hasFoundPerson:
        rule_based_planner.execution_order('find_person',
                person=task.person1, source=task.source, person_info = task.person2)
    
    # See if we need to ask what to say
    if task.whattosay == "answer a question":
        task.whattosay = ""
        rule_based_planner.execution_order('ask_question', voice_command="You can ask your question", output_variable="question")

        if not rule_based_planner.action_success:
            return

    # Ask the question to the executor
    rule_based_planner.execution_order('answer',
                                       voice_command=task.whattosay if task.whattosay != "" else "{question}")


def move_task(rule_based_planner, task): #! set a variable with current location?
    rule_based_planner.execution_order('move', destination=task.destination)


def find_obj_task(rule_based_planner, task):
    rule_based_planner.execution_order('find_object', 
                                        object=task.object1, object_info=task.object2,
                                        source=task.destination, output_variable="found_object")
    if rule_based_planner.action_success:
        rule_based_planner.has_found_object = True
    else:
        rule_based_planner.has_found_object = False

def find_person_task(rule_based_planner, task):
    rule_based_planner.execution_order('find_person', 
                                       person=task.person1, 
                                       source=task.destination, 
                                       person_info = task.person2, 
                                       output_variable="found_person")
    if rule_based_planner.action_success:
        rule_based_planner.hasFoundPerson = True
    else:
        rule_based_planner.hasFoundPerson = False


def find_task(rule_based_planner, task):
    if(task.object1 != ''):
        find_obj_task(rule_based_planner, task)
    if(task.person1 != ''):
        find_person_task(rule_based_planner, task)


def guide_task(rule_based_planner, task):
    # Find the person to guide
    if not rule_based_planner.hasFoundPerson:
        rule_based_planner.execution_order('find_person', person=task.person1, source=task.source, person_info = task.person2, output_variable="found_person")
        if rule_based_planner.action_success:
            rule_based_planner.hasFoundPerson = True
        else:
            rule_based_planner.hasFoundPerson = False

    # Ask for destination if none is provided
    if task.destination == "":
        rule_based_planner.execution_order('ask_question', voice_command="Where do you want to go?", output_variable="guide_dest")

        if not rule_based_planner.action_success:
            return
            
    # Guide the person
    rule_based_planner.execution_order('guide', 
                                       source=task.source, 
                                       destination=task.destination if task.destination != "" else "{guide_dest}", 
                                       person="{found_person}")
    
    # Assumes end of interaction
    rule_based_planner.hasFoundPerson = False
    

def introduce_task(rule_based_planner, task):
    guide_task(rule_based_planner, task)
    rospy.logwarn(task.destination)
    rule_based_planner.execution_order('find_person', person=task.person2, source=task.destination, voice_command = 'Hello ' + task.person2.rsplit(' ',1)[-1] + ', this is '+ task.person1)


def meet_task(rule_based_planner, task):
    if not rule_based_planner.hasFoundPerson:
        find_person_task(rule_based_planner, task)

    rule_based_planner.execution_order('speak',
        voice_command=f"Hello {task.person1} it's nice to meet you")


def telldescribe(rule_based_planner, task):
    if task.person1 != "":
        if not rule_based_planner.hasFoundPerson:
            pseudo_task = copy.deepcopy(task)
            pseudo_task.destination = pseudo_task.source
            pseudo_task.person1 = "person"
            pseudo_task.person2 = task.person1
            find_person_task(rule_based_planner, pseudo_task)

        rule_based_planner.execution_order('describe_person', person="{found_person}", 
                                        person_info="name,age,gender", output_variable="person_desc")

        # tell the operator
        if task.person2 == "operator":
            rule_based_planner.execution_order('move', destination=OPERATOR_LOC)

        # We have a destination, go there and find a person
        else:
            rule_based_planner.execution_order('find_person', 
                                               source=task.destination, 
                                               person="person", 
                                               person_info=task.person2 if task.person2 != "person" else "")

        # Say the description
        rule_based_planner.execution_order('speak', voice_command="{person_desc}")

    else:
        # Go to the source and look at the objects
        rule_based_planner.execution_order('find_objects', source=task.source, object=task.object1,
                                        output_variable="found_objects")
        rule_based_planner.execution_order('describe_objects', object="{found_objects}", 
                                        output_variable="object_desc")
        # tell the operator
        if task.person2 == "operator":
            rule_based_planner.execution_order('move', destination=OPERATOR_LOC)

        # We have a destination, go there and find a person
        else:
            rule_based_planner.execution_order('find_person', 
                                               destination=task.destination, 
                                               person="person", 
                                               person_info=task.person2 if task.person2 != "person" else "")
            
        # Say the description
        rule_based_planner.execution_order('speak', voice_command="{object_desc}")


def greet_task(rule_based_planner, task):
    rule_based_planner.execution_order('find_person',
        person=task.person1, person_info=task.person2,
        source=task.destination, output_variable="found_person")
    
    rule_based_planner.hasFoundPerson = True 
    
    rule_based_planner.execution_order("speak",
        voice_command="Hello. I'm BOB from Socrob!",)


def follow_task(rule_based_planner, task):
    if not rule_based_planner.hasFoundPerson:
        pseudo_task = copy.deepcopy(task)
        pseudo_task.destination = pseudo_task.source
        find_person_task(rule_based_planner, pseudo_task)
    rule_based_planner.execution_order('follow', person="{found_person}", destination=task.destination)
    rule_based_planner.hasFoundPerson = False #assumes end of interaction


def tellcount_task(rule_based_planner, task):
    if task.person1 != '':
        # Go to the source and look at the objects
        rule_based_planner.execution_order('find_people', source=task.destination, person=task.person1,
                                        person_info=task.person2, output_variable="found_people")
        rule_based_planner.execution_order('count', object="{found_people}", output_variable="number")
        # tell the operator
        rule_based_planner.execution_order('move', destination=OPERATOR_LOC)
            
        # Say the description
        rule_based_planner.execution_order('speak',
            voice_command=f"There are {{number}} people in the {task.destination} that match your description.")

    elif task.object1 != '':
        # Go to the source and look at the objects
        rule_based_planner.execution_order('find_objects', source=task.destination, object=task.object1,
                                        object_info=task.object2, output_variable="found_objects")
        rule_based_planner.execution_order('count', object="{found_objects}", output_variable="number")
        # tell the operator
        rule_based_planner.execution_order('move', destination=OPERATOR_LOC)
            
        # Say the description
        rule_based_planner.execution_order('speak',
            voice_command=f"There are {{number}} objects in the {task.destination} that match your description.")
        

def tellobject_task(rule_based_planner, task):
    rule_based_planner.execution_order('find_object', 
                                    object=task.object1, object_info=task.object2,
                                    source=task.source, output_variable="found_object")
    
    rule_based_planner.execution_order('describe_object', object="{found_object}", 
                                       object_info="name", output_variable="object_desc")
    
    # tell the operator
    rule_based_planner.execution_order('move', destination=OPERATOR_LOC)

    # Say the description
    rule_based_planner.execution_order('speak',
        voice_command=f"The {task.object2} {task.object1} in the {task.source} is {{object_desc}}.")
    

def tellmeet_task(rule_based_planner, task):
    if not rule_based_planner.hasFoundPerson:
        pseudo_task = copy.deepcopy(task)
        pseudo_task.destination = pseudo_task.source
        pseudo_task.person2 = ""
        find_person_task(rule_based_planner, pseudo_task)

    rule_based_planner.execution_order('describe_person', person="{found_person}", 
                                       person_info=task.whattosay, output_variable="person_desc")

    # No destination -> tell the operator
    if task.destination == "":
        rule_based_planner.execution_order('move', destination=OPERATOR_LOC)

    # We have a destination, go there and find a person
    else:
        rule_based_planner.execution_order('find_person', person=task.person2, source=task.destination)

    # Say the description
    rule_based_planner.execution_order('speak', voice_command=f"I have found a {{person_desc}} in the {task.source}.")


def askleave_task(rule_based_planner, task):
    task.destination = ASK_LEAVE_LOC
    meet_task(rule_based_planner, task)
    rule_based_planner.execution_order('speak',voice_command = task.person1 + ' I was told to ask you to leave. Please follow me to the exit.')
    rule_based_planner.execution_order('guide', source=task.source, destination=task.destination, person="{found_person}")
    rule_based_planner.hasFoundPerson = False #assumes end of interaction


def closedoor_task(rule_based_planner, task):
    rule_based_planner.execution_order('closedoor',source = task.destination)


def opendoor_task(rule_based_planner, task):
    rule_based_planner.execution_order('opendoor',source = task.destination)


# def place_in_task(rule_based_planner, task, pick_status):
#     if not(pick_status):
#         pseudo_task = copy.deepcopy(task)
#         pseudo_task.source = ""
#         pick_task(rule_based_planner, pseudo_task)
#     rule_based_planner.execution_order('place_in', object=task.object1, object2=task.object2, source=task.source, voice_command=task.object)


def takeout_task(rule_based_planner, task):
    if task.source == "":
        task.source = TAKEOUT_FIND_GARBAGE
    if task.destination == "":
        task.destination = TAKEOUT_PLACE
    if task.object1 == "garbage" or task.object1 == "":
        task.object1 = TAKEOUT_OBJ_NAME

    rule_based_planner.execution_order('find_object', 
                                        object=task.object1, object_info=task.object2,
                                        source=task.source, output_variable="found_object")
    rule_based_planner.execution_order('pick', object="{found_object}")
    rule_based_planner.execution_order('place', destination=task.destination, object="{found_object}")


def pour_task(rule_based_planner, task):
    if task.destination == "":
        task.destination = POUR_PLACE 

    # Find receptacle
    rule_based_planner.execution_order('find_object', object=task.object1,
                                        source=task.source, output_variable="object1")
    rule_based_planner.execution_order('pick', object="{object1}")
    rule_based_planner.execution_order('place', destination=task.destination, object="{object1}")

    # Find the other object
    rule_based_planner.execution_order('find_object', object=task.object2,
                                        source=task.source, output_variable="object2")
    rule_based_planner.execution_order('pick', object="{object2}")
    rule_based_planner.execution_order('place', destination=task.destination, object="{object2}")
    rule_based_planner.execution_order('pour', object="{object1}", object_info="{object2}")


def serve_task(rule_based_planner, task):
    # if task.object1 == "eat":
    #     task.object1 = SERVE_EAT

    # if task.object1 == "drink":
    #     task.object1 = SERVE_DRINK

    # rule_based_planner.execution_order('local_find_person_serve', person=task.person1, person_info=task.person2, source=task.destination)
    # rule_based_planner.execution_order('local_find_obj_serve', object=task.object1)
    # rule_based_planner.execution_order('move', destination=task.destination)
    # rule_based_planner.execution_order('speak',voice_command='I have brought you some ' + task.object1 + '.' + 'Please, ' + task.person1 + ' get the items from my rightside carrier.')
    # rule_based_planner.execution_order('speak',voice_command='I will wait 20 seconds. Then move on to a new task.')
    # rule_based_planner.execution_order('serve', person_info='20', voice_command='Please remove your items!')
    pass


def cleanup_task(rule_based_planner, task):
    pass