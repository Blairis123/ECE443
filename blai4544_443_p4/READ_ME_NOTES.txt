	Everything that is wrong with the project
Nonfunctional I2C bus (reading from IR sensor using I2C results in a general exception handler)
Nonfunctional message buffer (Tx/Rx always result in a 0 or NULL value)

	Everything that is correct with the project
The task notification works
Setting the CN ISR flag works
Task priorities should be set as specified in proj requirements
String's format is correct

	What I think is correct but couldn't find a way to test
I think I set up the pointer to the local beffer correctly (the email helped! Thank you Dr.J)