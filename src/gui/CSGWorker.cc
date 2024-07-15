#include "CSGWorker.h"
#include <QThread>

#include "Tree.h"
#include "GeometryEvaluator.h"
#include "progress.h"
#include "printutils.h"
#include "exceptions.h"

CSGWorker::CSGWorker(MainWindow *main)
{
  this->main = main;
  this->started=0;
  this->thread = new QThread();
  if (this->thread->stackSize() < 1024 * 1024) this->thread->setStackSize(1024 * 1024);
  connect(this->thread, SIGNAL(started()), this, SLOT(work()));
  moveToThread(this->thread);
}

CSGWorker::~CSGWorker()
{
  delete this->thread;
}

int CSGWorker::start(void)
{
  if(started) return 0;	
  started=1;
  this->thread->start();
  return 1;
}

void CSGWorker::work()
{
  // this is a worker thread: we don't want any exceptions escaping and crashing the app.
  try {
    main->compileCSGThread();
  } catch (const ProgressCancelException& e) {
    LOG("Compilation cancelled.");
  } catch (const HardWarningException& e) {
    LOG("Compilation cancelled on first warning.");
  } catch (const std::exception& e) {
    LOG(message_group::Error, "Compilation cancelled by exception %1$s", e.what());
  } catch (...) {
    LOG(message_group::Error, "Compilation cancelled by unknown exception.");
  }

  emit done();
  this->started=0;
  thread->quit();
}
