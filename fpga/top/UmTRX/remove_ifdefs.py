import sys
import os

CAREABOUT = ['NO_SERDES', 'UMTRX', 'LMS602D_FRONTEND', 'LMS_DSP']

if __name__ == '__main__':
    filePath = sys.argv[1]
    outLines = []

    pp_stack = []

    for line in open(filePath).readlines():
        if line.startswith('`ifndef'):
            what = line.split()[1].strip()
            pp_stack.append(['ifndef', what])
        if line.startswith('`ifdef'):
            what = line.split()[1].strip()
            pp_stack.append(['ifdef', what])
        if line.startswith('`else'):
            if pp_stack[-1][0] == 'ifdef': pp_stack[-1][0] = 'ifndef'
            if pp_stack[-1][0] == 'ifndef': pp_stack[-1][0] = 'ifdef'

        if not pp_stack:
            outLines.append(line)
        elif pp_stack[-1][1] not in CAREABOUT:
            outLines.append(line)
        elif line.startswith('`ifndef'):
            pass
        elif line.startswith('`ifdef'):
            pass
        elif line.startswith('`else'):
            pass
        elif line.startswith('`endif'):
            pass
        elif pp_stack[-1][0] == 'ifdef':
            outLines.append(line)

        if line.startswith('`endif'):
            pp_stack = pp_stack[:-1]

    assert(not pp_stack)

    open(filePath, 'w').write(''.join(outLines))
