package com.irurueta.android.navigation.inertial

import java.util.concurrent.TimeUnit
import java.util.concurrent.locks.ReentrantLock
import kotlin.concurrent.withLock

class ThreadSyncHelper {
    private val lock = ReentrantLock()
    private val condition = lock.newCondition()

    fun waitOnCondition(
        condition: () -> Boolean,
        maxRetries: Int = MAX_RETRIES,
        timeout: Long = TIMEOUT
    ) {
        lock.withLock {
            var count = 0
            while (condition() && count < maxRetries) {
                this.condition.await(timeout, TimeUnit.MILLISECONDS)
                count++
            }
        }
    }

    fun notifyAll(lambda: () -> Unit) {
        lock.withLock {
            lambda()
            condition.signalAll()
        }
    }

    private companion object {
        private const val MAX_RETRIES = 2
        private const val TIMEOUT = 20000L
    }
}