import numpy
from numpy import sin, cos

def rotation(axis, amount):
    if axis == 2:
        R = numpy.array([[cos(amount), -sin(amount), 0],
                         [sin(amount),  cos(amount), 0],
                         [0, 0, 1]])
    elif axis == 1:
        R = numpy.array([[cos(amount), 0, -sin(amount)],
                         [0, 1, 0],
                         [sin(amount),  0, cos(amount)]])
    elif axis == 0:
        R = numpy.array([[1, 0, 0],
                         [0, cos(amount), -sin(amount)],
                         [0, sin(amount), cos(amount)]])
    return R


def flatmeshgrid(*args, **kw):
    return [numpy.ravel(x) for x in numpy.meshgrid(*args, **kw)]

def raster_2d(xs, ys):
    n = len(xs)
    m = len(ys)
    out = numpy.zeros((m, n, 2))
    for i, y in enumerate(ys):
        out[i,:,1] = y
        if i % 2 == 0:
            out[i,:,0] = xs
        else:
            out[i,:,0] = xs[::-1]
    out = out.reshape((-1, 2))
    return out[:,0], out[:,1]

