///////////////////////////////////////////////////////////////////             RECEIVE BYTE
void receiveBytes()
{
  static byte index = 0;
  char terminator = '\r'; // what tells us our command is done
  while (Serial.available() > 0)
  {
    rcv_buffer[index] = Serial.read(); // read in our serial commands
    if (rcv_buffer[index] == terminator) // main loop for processing our command
    {
      index = 0;
      parseCommand();
      buffer_Flush(rcv_buffer);
    }
    else
    {
      index++;
      if (index >= 64)
      {
        Serial.println("buffer overflow");
        index = 0;
        buffer_Flush(rcv_buffer);
      }
    }
  }
}

///////////////////////////////////////////////////////////////////            BUFFER FLUSH
void buffer_Flush(char *ptr)
{
  for (int i = 0; i < 64; i++)
  {
    ptr[i] = 0;
  }
}

