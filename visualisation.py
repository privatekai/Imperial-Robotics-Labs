corners = [(0, 0), (40, 0), (40, 40), (0, 40)]
lines = []


for i in range(1, len(corners) + 1):
    lines.append(corners[i - 1] + corners[i % len(corners)])

for line in lines:
    print("drawLine:", str(line))
