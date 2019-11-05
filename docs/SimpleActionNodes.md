# Decorators

A decorator is a node that can have only a single child.

It is up to the Decorator to decide if, when and how many times the child should be
ticked.

## RateController

Tick the child up to N times, where N is passed as a [Input Port](tutorial_02_basic_ports.md),
as long as the child returns SUCCESS.

Interrupt the loop if the child returns FAILURE and, in that case, return FAILURE too.

If the child returns RUNNING, this node returns RUNNING too.

## RepeatUntil

If the child returns RUNNING, this node returns RUNNING too. 

Otherwise, it returns always SUCCESS.

## Forever

If the child returns RUNNING, this node returns RUNNING too. 

Otherwise, it returns always FAILURE.
