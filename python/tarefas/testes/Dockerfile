FROM python:3.12-slim
WORKDIR /s500
COPY gazebo gazebo
COPY python python
RUN python -m compileall -q gazebo/windows python
CMD ["sh", "-c", "python python/tarefas/testes/test_missoes_2027.py && python python/tarefas/testes/sitl/tests/test_plan.py"]
