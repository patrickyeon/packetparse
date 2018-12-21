#!/usr/bin/python
# generates (importable) python files out of JSON constant files
# this is actually cleaner when you need to import packetparse.py

with open("constants.json", "r") as c_in:
    with open("constants.py", "w") as c_out:
        c_out.write("constants = " + c_in.read())

with open("mappings.json", "r") as m_in:
    with open("mappings.py", "w") as m_out:
        m_out.write("mappings = " + m_in.read())