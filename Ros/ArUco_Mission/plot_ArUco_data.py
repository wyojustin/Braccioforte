'''Not intended for Rpi usage!'''
from __future__ import print_function
from mpl_toolkits.mplot3d import Axes3D 
from numpy import *
from pylab import *
from StringIO import StringIO

def err_plot(ax, meas, true, *args, **kw):
    n = len(meas)
    assert len(true) == n
    ads = linalg.norm(true - meas, axis=1)
    thresh = percentile(ads, 25)
    keep = ads < thresh
    meas = meas[keep]
    true = true[keep]
    n = len(meas)
    xx = zeros((n, 3))
    yy = zeros((n, 3))
    xx[:,0] = meas[:,0]
    xx[:,1] = true[:,0]
    xx[:,2] = nan

    yy[:,0] = meas[:,1]
    yy[:,1] = true[:,1]
    yy[:,2] = nan


    ax.plot(xx.ravel(), yy.ravel(), *args, **kw)
    # mn = mean(meas - true, axis=1)
    # ax.plot(mn[0], mn[1], 'bd', markersize=10)
    
class Struct:
    def __init__(self, **kw):
        self.kw = kw
    def __getattr__(self, key):
        return self.kw[key]
    
def parsedata(fn):
    txt = [l for l in open(fn).readlines()[:-1] if len(l) > 90]
    dat = loadtxt(StringIO(''.join(txt)))
    dat = dat[dat[:,9] < 1]
    dat = dat[dat[:,9] > .3]
    n = len(dat)
    values = dat[:,0].astype(int)
    pos = dat[:,1:4]
    cam = dat[:,4:7]
    offset_cam = dat[:,7:10]
    orient = dat[:,10:19].reshape((-1, 3, 3))
    dist = linalg.norm(pos - cam, axis=1)
    true_lookup = {}
    for v in list(set(values)):
        true_lookup[v] = mean(pos[values == v], axis=0)
    true_offset = empty((n, 3))
    for i in range(n):
        true_offset[i] = dot(orient[i].T, true_lookup[values[i]] - cam[i])
    return Struct(values=values, pos=pos, cam=cam, offset_cam=offset_cam, orient=orient, true_offset=true_offset, dist=dist)
fn = 'cal_data_4.txt'
dat = parsedata(fn)

fig = figure(figsize=(12, 6))
ax = fig.add_subplot(121, projection='3d')
ax.set_xlabel('$x$', fontsize=24)
ax.set_ylabel('$y$', fontsize=24)
ax.set_zlabel('$z$', fontsize=24)
ax.axis('equal')
values = dat.values
pos = dat.pos
cam = dat.cam
orient = dat.orient

true_pos = {}
for v in list(set(values)):
    true_pos[v] = mean(pos[values == v], axis=0)


c = arctan2(cam[:,0], cam[:,1])
s = 3
ax.scatter(pos[:,0], pos[:,1], pos[:,2], s=s, c=c, edgecolor='none')
ax.scatter(cam[:,0], cam[:,1], cam[:,2], s=s, c=c, edgecolor='none')

ax = fig.add_subplot(122)
ax.axis('equal')
ax.set_xlabel('$x$', fontsize=24)
ax.set_ylabel('$y$', fontsize=24)
ax.scatter(pos[:,0], pos[:,1], s=s, c=c, edgecolor='none')
# ax.scatter(cam[:,0], cam[:,1], s=s, c=c, edgecolor='none')
for v in true_pos:
    ax.plot(true_pos[v][0], true_pos[v][1], 'kd')

axis('equal')
vals = [[42, 18, 12],
        [27, 43, 5]]
figure()
ax = gca()
axis('equal')
err_plot(ax, dat.offset_cam[:,0:2]/dat.dist[:,newaxis], dat.true_offset[:,0:2]/dat.dist[:,newaxis], 'r-')

fig, ax = subplots(2, 3, sharex=True, sharey=True)
ax[0,0].axis('equal')

for i, row in enumerate(vals):
    for j, v in enumerate(row):
        keep_v = dat.values == v
        ax[i][j].set_title(str(v))
        err_plot(ax[i][j], dat.offset_cam[keep_v,0:2] / dat.dist[keep_v,newaxis], dat.true_offset[keep_v,0:2]/ dat.dist[keep_v,newaxis], 'r-')
show()
