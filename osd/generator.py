import re

# Quick and dirty implementation

f = open('symbols.h')
text = f.read()

SIZE = 8

alt_names = {' ' : 'SPACE',
             '-' : 'DASH',
             '/' : 'SLASH',
             ':' : 'COLON',
             '%' : 'PERCENT',
             '.' : 'DOT'}

p = re.search('unsigned\s*char\s*SYMBOLS', text)
text = text[p.start():]
text = text.split('{')[1].split('}')[0]
lines = text.split('\n')
i = 0
for line in lines:
    ln = line.strip()
    if (ln.startswith('//')):
        ln = ln[2:].strip()
        if (len(ln) > 2):
            if (ln[0] == "'" and ln[2] == "'"):
                ch = ln[1]
                if (ch in alt_names):
                    ch = alt_names[ch]
                print("#define _{0} {1}".format(ch, i * SIZE))
                i += 1

f.close()
