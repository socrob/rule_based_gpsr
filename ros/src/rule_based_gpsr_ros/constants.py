# For each NLU verb, the list of actions that verb depends on
# If you want to block a verb just comment it.
ACTIONS_IN_VERB = {
    "take": [
        "find_object",
        "pick",
    ], 
    "place": [
        "find_object",
        "pick",
        "place",
    ], 
    "bring": [
        "find_object",
        "pick",
        "move",
        "find_person",
        "deliver",
    ], 
    "deliver": [
        "find_object",
        "pick",
        "move",
        "find_person",
        "deliver",
    ], 
    "speak": [
        "find_person",
        "ask_question",
        "answer",
    ], 
    "gopl": [
        "move",
    ], 
    "find": [
        "find_object",
        "find_person",
    ], 
    "guide": [
        "find_person",
        "ask_question",
        "guide",
    ], 
    "meet": [
        "find_person",
        "speak",
    ], 
    "greet": [
        "find_person",
    ], 
    "introduce": [
        "find_person",
        "ask_question",
        "guide",
        "find_person",
    ], 
    "tellcount": [
        "find_people",
        "find_objects",
        "count",
        "move",
        "speak",
    ], 
    "tellmeet": [
        "find_person",
        "describe_person",
        "move",
        "speak",
    ], 
    "tellobject": [
        "find_object",
        "describe_object",
        "move",
        "speak",
    ], 
    "follow": [
        "find_person",
        "follow",
    ], 
    "takeout": [
        "find_object",
        "pick",
        "place",
    ], 
    "askleave": [
        "find_person",
        "speak",
        "guide",
    ], 
    "opendoor": [
        "opendoor"
    ], 
    "closedoor": [
        "closedoor"
    ], 
    # "place_in": [
    #     "find_object",
    #     "place_in",
    # ], 
    "pour": [
        "find_object",
        "pick",
        "place",
        "pour",
    ], 
    "telldescribe": [
        "find_person",
        "describe_person",
        "move",
        "speak",
    ], 
    # "serve": [
    # ], 
    # "cleanup": [
    # ], 
    "end_speech": [],
}

