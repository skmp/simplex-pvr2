// Measure fmac_seq (IP-path stub) latency & verify result alignment.
#include <verilated.h>
#include "Vfmac_seq.h"
#include <cstdio>
#include <cstring>
#include <cstdint>
static uint32_t fb(float f){uint32_t u;memcpy(&u,&f,4);return u;}
static float af(uint32_t u){float f;memcpy(&f,&u,4);return f;}
int main(int c,char**v){
    Verilated::commandArgs(c,v);
    auto*d=new Vfmac_seq;
    auto tick=[&](){d->clk=0;d->eval();d->clk=1;d->eval();};
    d->reset=1;d->clk=0;d->req=0;d->eval();tick();tick();d->reset=0;
    // op: 3.0 * 4.0 + 1.0 = 13.0 ; sub=0 neg=0
    d->a=fb(3.0f);d->b=fb(4.0f);d->c=fb(1.0f);d->sub=0;d->neg_p=0;
    d->req=1;tick();d->req=0;
    int ackcyc=-1;
    for(int i=0;i<40;i++){ tick(); if(d->ack){ackcyc=i+1;printf("ack at cycle %d after req, q=%.6g (%08x)\n",ackcyc,af(d->q),d->q);break;} }
    if(ackcyc<0)printf("NO ACK in 40 cycles\n");
    // expect 13.0
    printf("expect 13.0\n");
    delete d;return 0;
}
