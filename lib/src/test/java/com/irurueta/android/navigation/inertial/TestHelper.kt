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

import kotlin.reflect.KClass
import kotlin.reflect.KMutableProperty
import kotlin.reflect.full.declaredFunctions
import kotlin.reflect.full.declaredMemberFunctions
import kotlin.reflect.full.declaredMemberProperties
import kotlin.reflect.jvm.isAccessible
import kotlin.reflect.jvm.javaField

@Suppress("UNCHECKED_CAST")
inline fun <reified T : Any, R> callPrivateFuncWithResult(
    c: KClass<T>,
    obj: T,
    name: String,
    vararg args: Any?
): R? {
    return callPrivateFunc(c, obj, name, *args) as? R
}

inline fun <reified T : Any, R> T.callPrivateFuncWithResult(name: String, vararg args: Any?): R? =
    callPrivateFuncWithResult(T::class, this, name, *args)

@Suppress("UNCHECKED_CAST")
inline fun <reified T : Any, R> callPrivateStaticFuncWithResult(
    c: KClass<T>, obj: T, name: String,
    vararg args: Any?
): R? {
    return callPrivateStaticFunc(c, obj, name, *args) as? R
}

inline fun <reified T : Any, R> T.callPrivateStaticFuncWithResult(
    name: String,
    vararg args: Any?
): R? = callPrivateStaticFuncWithResult(T::class, this, name, *args)

inline fun <reified T : Any> callPrivateFunc(
    c: KClass<T>,
    obj: T,
    name: String,
    vararg args: Any?
): Any? {
    return c.declaredMemberFunctions
        .firstOrNull { it.name == name }
        ?.apply { isAccessible = true }
        ?.call(obj, *args)
}

inline fun <reified T : Any> T.callPrivateFunc(name: String, vararg args: Any?): Any? =
    callPrivateFunc(T::class, this, name, args)

inline fun <reified T : Any> callPrivateStaticFunc(
    c: KClass<T>, obj: T, name: String,
    vararg args: Any?
): Any? {
    return c.declaredFunctions
        .firstOrNull { it.name == name }
        ?.apply { isAccessible = true }
        ?.call(obj, *args)
}

inline fun <reified T : Any> T.callPrivateStaticFunc(name: String, vararg args: Any?): Any? =
    callPrivateStaticFunc(T::class, this, name, args)

@Suppress("UNCHECKED_CAST")
inline fun <reified T : Any, R> getPrivateProperty(c: KClass<T>, obj: T, name: String): R? {
    return c.declaredMemberProperties
        .firstOrNull { it.name == name }
        ?.apply { isAccessible = true }
        ?.get(obj) as? R
}

inline fun <reified T : Any, R> T.getPrivateProperty(name: String): R? =
    getPrivateProperty(T::class, this, name)

inline fun <reified T : Any, R> setPrivateProperty(c: KClass<T>, obj: T, name: String, value: R?) {
    val property = c.declaredMemberProperties.find { it.name == name }
    if (property is KMutableProperty<*>) {
        property.isAccessible = true
        property.setter.call(obj, value)
    } else {
        property?.isAccessible = true
        property?.javaField?.isAccessible = true
        property?.javaField?.set(obj, value)
    }
}

inline fun <reified T : Any, R> T.setPrivateProperty(name: String, value: R?) {
    setPrivateProperty(T::class, this, name, value)
}
