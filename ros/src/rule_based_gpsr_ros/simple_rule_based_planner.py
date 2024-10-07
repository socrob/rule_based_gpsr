#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from tiago_executor.msg import action_msg
from google_speech_recognition.msg import ASRNBestList, ASRHypothesis
from nlu_rule_based.msg import nlu_msg

from rule_based_gpsr_ros.rule_based_planner_funcs import convert_task_to_actions

class LLMNode():
    def __init__(self):
        # Init node
        rospy.init_node("rule_based_planner", anonymous=False)

        # Subscriber for new instructions and event_in
        rospy.Subscriber("~instruction", String, callback=self.generate_plan, queue_size=5)
        rospy.Subscriber("~event_in", String, callback=self.event_in, queue_size=5)
        
		# Publisher to send a instruction to the NLU and Subscriber to receive the result
        self.nlu_publisher = rospy.Publisher(rospy.get_param("~nlu_transcript_topic", "/mbot_speech_recognition/transcript"),
										     ASRNBestList, queue_size = 5)
        rospy.Subscriber(rospy.get_param("~nlu_result_topic", "/nlu_rule_based/dialogue_acts"),
                                         nlu_msg, self.nlu_results_callback, queue_size = 5)
        
        # Publisher for actions
        self.action_publisher = rospy.Publisher("~actions", action_msg, queue_size=5)

        # Variables needed for planning
        self.manip_flag = False
        self.hasFoundPerson = False
        self.has_found_object = False
        self.picked_value = False

        # This nodes lives in the wonderful world where everything is a success
        self.action_success = True

        rospy.loginfo("Rule Based Planner initialized!")
        rospy.set_param("~ready", True)

    def generate_plan(self, instruction: String):
        rospy.loginfo(f"Sending instruction '{instruction.data}' to the NLU")
        # Send instruction to the NLU, the NLU callback will convert the NLU tasks to actions
        # and publish them on the result topic
        transcript = ASRNBestList()
        transcript.hypothesis.append(ASRHypothesis(transcript=instruction.data,
                                     confidence=1))
        
        self.nlu_publisher.publish(transcript)
            
    def nlu_results_callback(self, msg: nlu_msg):
        # If we reached the end publish an end_execution message
        if msg.verb == "end_speech":
            rospy.loginfo("Received end of speech message from NLU")
            self.action_publisher.publish(action_msg(name="end_execution"))

            # Reset the state variables
            self.manip_flag = False
            self.hasFoundPerson = False
            self.has_found_object = False
            self.picked_value = False
        
        # If not generate a plan for this task
        else:
            convert_task_to_actions(self, msg)

    def execution_order(self,
                        action, 
                        source = '', 
                        destination='', 
                        person = '', 
                        person_info='', 
                        object='', 
                        object_info='', 
                        voice_command='',
                        output_variable=''):
        '''
        Publishes an action to the plan
        '''
        msg = action_msg(
            name=action,
            source=source,
            destination=destination,
            object=object,
            object_info=object_info,
            person=person,
            person_info=person_info,
            voice_command=voice_command,
            output_variable=output_variable,
        )
        self.action_publisher.publish(msg)

    def event_in(self, msg: String):
        # Event in is only here to maintain the same interface as the llm planner
        # Since this node does not use complex models there is no need to turn it on and off
        return

if __name__ == "__main__":
    node = LLMNode()
    rospy.spin()
    