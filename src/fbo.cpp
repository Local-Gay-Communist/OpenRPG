// fbo.cpp
#include "fbo.h"
#include "globals.h"
#include <GL/glew.h>
#include <cstdio>

void init_fbo(int w, int h) {
    viewport_width = w;
    viewport_height = h;
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    glGenTextures(1, &fbo_texture);
    glBindTexture(GL_TEXTURE_2D, fbo_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fbo_texture, 0);

    glGenRenderbuffers(1, &fbo_renderbuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, fbo_renderbuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, w, h);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, fbo_renderbuffer);

    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        printf("FBO error!\n");
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}