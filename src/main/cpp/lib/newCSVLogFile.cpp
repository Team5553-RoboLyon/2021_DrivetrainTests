#include "lib/newCSVLogFile.h"

newCSVLogFile::newCSVLogFile(char *filename, char *head)
{
    m_file = fopen(filename, "w+");
    fwrite(head, sizeof(char), sizeof(head), m_file);
}

newCSVLogFile::~newCSVLogFile()
{
    fclose(m_file);
}
void newCSVLogFile::log(double value[3], int arraySize)

{
    char monChar[17] = {0};
    sprintf(monChar, "%.15lf", value[0]);
    fwrite(monChar, sizeof(char), 17, m_file);
    for (int i = 1; i < arraySize; i++)
    {
        char comma[1] = {','};
        fwrite(comma, sizeof(char), 1, m_file);
        sprintf(monChar, "%.15lf", value[i]);
        fwrite(monChar, sizeof(char), 17, m_file);
    }
    char end[1] = {'\n'};
}