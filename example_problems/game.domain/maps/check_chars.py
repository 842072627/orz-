lines=open('orz900d.map').readlines()
h=next(i for i,l in enumerate(lines) if l.strip()=='map')+1
cols=1491

# Check characters around (96,539) from algoPts
r=539
print(f'Row {r} cols 92-102: {repr(lines[h+r][92:102])}')

# All unique non-@ chars
chars=set()
for l in lines[h:h+656]:
    for c in l.strip():
        if c!='@':
            chars.add(c)
print(f'Non-wall chars: {chars}')

# Check first rows of corridor
for r in range(468, 475):
    print(f'Row {r} cols 0-30: {repr(lines[h+r][0:30])}')
