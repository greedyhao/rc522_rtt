from building import * 
import os
import rtconfig

# get current dir path
cwd = GetCurrentDir()

src  = Glob('src/*.c')
src  += Glob('port/*.c')
inc = [cwd]
inc += [cwd + '/inc']

if GetDepend(['PKG_USING_RC522_SAMPLE']):
    src += Glob('examples/*.c')

group = DefineGroup('rc522', src, depend = ['PKG_USING_RC522'], CPPPATH = inc)
Return('group')
