import numpy as np
import ocelot.djinni.Wave as w
import math as math

x = np.array([[5, 0], [0, 5]], dtype=complex)
y = np.array([[5, 0], [0, 5]], dtype=complex)

print(len(x.tobytes()))

r = w.Wave.mul(x.tobytes(), y.tobytes())

print(r)
print(len(r))

result = np.frombuffer(r, dtype=complex)

print(result)

size = int(math.sqrt(len(result)))
r2 = result.reshape((size, size))

print(r2)
