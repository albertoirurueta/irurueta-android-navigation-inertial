/*
 * Copyright (C) 2021 Alberto Irurueta Carro (alberto@irurueta.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
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