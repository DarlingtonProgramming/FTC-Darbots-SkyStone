package org.darbots.darbotsftclib.libcore.templates.log;

import android.support.annotation.NonNull;
import android.support.annotation.Nullable;

import org.darbots.darbotsftclib.libcore.integratedfunctions.logger.logContents.String_Log;

import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;

public abstract class LogContent implements Map<String, Object> {
    public abstract LogType getContentType();
    public abstract Object getContentValue();
    public abstract void setContentValue(Object contentVal);

    @Override
    public int size() {
        return 2;
    }

    @Override
    public boolean isEmpty() {
        return false;
    }

    @Override
    public boolean containsKey(@Nullable Object key) {
        boolean expression = key.equals("type") || key.equals("value");
        return expression;
    }

    @Override
    public boolean containsValue(@Nullable Object value) {
        Collection<Object> valueSet = this.values();
        return valueSet.contains(value);
    }

    @Nullable
    @Override
    public Object get(@Nullable Object key) {
        if(key.equals("type")){
            return this.getContentType().name();
        } else if (key.equals("value")) {
            return this.getContentValue();
        }
        return null;
    }

    @Nullable
    @Override
    public Object put(@NonNull String key, @NonNull Object value) {
        if(key.equals("value")){
            this.setContentValue(value);
        }
        return null;
    }

    @Nullable
    @Override
    public Object remove(@Nullable Object key) {
        return null;
    }

    @Override
    public void putAll(@NonNull Map<? extends String, ?> m) {
        for (Entry<? extends String, ?> i: m.entrySet()) {
            this.put(i.getKey(),i.getValue());
        }
    }

    @Override
    public void clear() {
        return;
    }

    @NonNull
    @Override
    public Set<String> keySet() {
        Set<String> mKeys = new TreeSet<>();
        mKeys.add("type");
        mKeys.add("value");
        return mKeys;
    }

    @NonNull
    @Override
    public Collection<Object> values() {
        Collection<Object> mValues = new ArrayList<>();
        mValues.add(this.getContentType().name());
        mValues.add(this.getContentValue());
        return mValues;
    }

    @NonNull
    @Override
    public Set<Entry<String, Object>> entrySet() {
        Set<Entry<String,Object>> mSet = new TreeSet();
        mSet.add(new AbstractMap.SimpleEntry<String, Object>("type", this.getContentType().name()));
        mSet.add(new AbstractMap.SimpleEntry<String, Object>("value", this.getContentValue()));
        return mSet;
    }

    @Nullable
    public static LogContent getInstance(String logType, Object value){
        if(logType.equals(LogType.STRING_LOG.name())){
            //Initialize a String Log
            return new String_Log(value);
        }else{
            return null;
        }
    }
}
