# Simple C++ compilation for ImageTracer
optimize = ['-O2'] # Was ['-g']
env = Environment(CPPFLAGS=['-Wall','-std=c++2a']+optimize)
env.ParseConfig('pkg-config --cflags --libs fmt')
env.Program('ImageTracer',
            ['ImageTracer.cpp'])
env.Program('ImageTracerBW',
            ['ImageTracerBW.cpp'])
