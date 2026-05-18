static int g_log_interval = 100;

static void init_mp_log() {
    g_mp_peak_density.assign(g_mp_count, 0);
    g_mp_peak_tick.assign(g_mp_count, -1);
}

static void record_mp_snapshot(const SharedEnvironment* env) {
    int tick = env->curr_timestep;
    if (tick % g_log_interval != 0) return;
    g_log_ticks.push_back(tick);
    g_log_density.push_back(g_mp_density);
    g_log_lock.push_back(g_mp_lock_dir);
    for (int si = 0; si < g_mp_count; si++) {
        if (g_mp_density[si] > g_mp_peak_density[si]) {
            g_mp_peak_density[si] = g_mp_density[si];
            g_mp_peak_tick[si] = tick;
        }
    }
}

// Registered via atexit, writes to workspace path
static int g_atexit_dummy = (std::atexit([](){
    const char* path = "C:/Users/Administrator/.openclaw/workspace/mp_analysis.json";
    FILE* f = fopen(path, "w");
    if (!f || g_log_ticks.empty()) { if(f)fclose(f); return; }
    fprintf(f, "{\n  \"mp_count\":%d,\n  \"log_interval\":%d,\n", g_mp_count, g_log_interval);
    fprintf(f, "  \"mp_locations\":[");
    for(int si=0;si<g_mp_count;si++){if(si)fprintf(f,",");fprintf(f,"%d",g_mp_loc[si]);}
    fprintf(f,"],\n  \"mp_peak_density\":[");
    for(int si=0;si<g_mp_count;si++){if(si)fprintf(f,",");fprintf(f,"%d",g_mp_peak_density[si]);}
    fprintf(f,"],\n  \"mp_peak_tick\":[");
    for(int si=0;si<g_mp_count;si++){if(si)fprintf(f,",");fprintf(f,"%d",g_mp_peak_tick[si]);}
    fprintf(f,"],\n  \"ticks\":[");
    for(size_t i=0;i<g_log_ticks.size();i++){if(i)fprintf(f,",");fprintf(f,"%d",g_log_ticks[i]);}
    fprintf(f,"],\n  \"history\":[\n");
    for(size_t hi=0;hi<g_log_ticks.size();hi++){
        if(hi)fprintf(f,",\n");
        fprintf(f,"    {\"tick\":%d,\"density\":[",g_log_ticks[hi]);
        for(int si=0;si<g_mp_count;si++){if(si)fprintf(f,",");fprintf(f,"%d",g_log_density[hi][si]);}
        fprintf(f,"],\"lock\":[");
        for(int si=0;si<g_mp_count;si++){if(si)fprintf(f,",");fprintf(f,"%d",g_log_lock[hi][si]);}
        fprintf(f,"]}");
    }
    fprintf(f,"\n  ]\n}\n");
    fclose(f);
}), 0);
#endif

} // namespace Centerline
