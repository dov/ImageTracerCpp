# Simple C++ compilation for ImageTracer
env = Environment(CPPFLAGS=['-Wall','-g','-std=c++2a'])
env.ParseConfig('pkg-config --cflags --libs fmt')
env.Program('ImageTracer',
            ['ImageTracer.cpp'])
