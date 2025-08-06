##
## Modify the knowledge base (add,retract,update statements)
##

string UPDATE=update
# alias for update
string ADD=update
string RETRACT=retract
# alias for RETRACT
string REMOVE=retract
# alias for RETRACT
string DELETE=retract

string method
string[] statements
string[] models

# only relevant for UPDATE. Lifespan of 0 (default) means no expiration.
builtin_interfaces/Duration lifespan
