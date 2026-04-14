#include <random>
#include "Entry.h"

// default planner τÜäσñ┤µûçΣ╗╢
#include "planner.h"
#include "const.h"

// ==================== MAPFPlanner::initialize ====================
// ΘóäσñäτÉåΘÿ╢µ«╡∩╝Üσê¥σºïσîû default planner τÜäµëÇµ£ëµò░µì«τ╗ôµ₧ä
// σîàµï¼∩╝ÜσÉ»σÅæσ╝ÅΦí¿πÇüTrajLNS∩╝êΦ╜¿Φ┐╣+µ╡üΣ╝ÿσîû∩╝ëπÇüPIBT µò░µì«τ╗ôµ₧äπÇüΣ╝ÿσàêτ║º
//
// µö╣Φ┐¢∩╝Üµ¢┤τ▓╛τí«τÜäµù╢Θù┤Φ«íτ«ù
// - τö¿ microseconds∩╝êΣ╕ìµÿ» milliseconds∩╝ëΦ«íτ«ùσ╖▓τö¿µù╢Θù┤∩╝îσçÅσ░æΦ»»σ╖«
// - Φ┐Öτí«Σ┐¥ planner Σ╕ìΣ╝Üσ¢áΣ╕║µù╢Θù┤Φ«íτ«ùΣ╕ìτ▓╛τí«ΦÇîΦ╢àµù╢
// τü╡µäƒµ¥ÑΦç¬ Kitty Knight∩╝ÜΣ╗ûΣ╗¼Σ╣ƒτö¿ microseconds µ¥Ñτ▓╛τí«µÄºσê╢µù╢Θù┤
void MAPFPlanner::initialize(int preprocess_time_limit)
{
    // τö¿σë⌐Σ╜ÖτÜäΘóäσñäτÉåµù╢Θù┤∩╝êσçÅσÄ╗ scheduler σ╖▓τ╗Åτö¿µÄëτÜä∩╝ëσê¥σºïσîû planner
    // - PLANNER_TIMELIMIT_TOLERANCE µÿ»σ«ëσà¿Σ╜ÖΘçÅ∩╝ê10ms∩╝ë
    // - τö¿ microseconds Φ«íτ«ùσ╖▓Φ┐çµù╢Θù┤∩╝îµ»ö milliseconds τ▓╛τí«1000σÇì
    int elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - env->plan_start_time).count();
    int limit = preprocess_time_limit - elapsed_us / 1000 
              - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;
    if (limit < 10) limit = 10;

    DefaultPlanner::initialize(limit, env);
}

// ==================== MAPFPlanner::plan ====================
// µ»ÅΣ╕¬ planning episode Φó½Φ░âτö¿Σ╕Çµ¼í
// µá╕σ┐âσ╖ÑΣ╜£Σ║ñτ╗Ö DefaultPlanner::plan∩╝îσ«âσüÜΣ║åΣ╕ëΣ╗╢Σ║ï∩╝Ü
//   1. µ¢┤µû░σ╝òσ»╝Φ╖»σ╛ä∩╝êguide path∩╝ë+ Frank-Wolfe µ╡üΣ╝ÿσîû
//   2. σñÜµ¡Ñ PIBT ΦºäσêÆ∩╝êµîëΣ╝ÿσàêτ║ºσå│σ«Üµ»ÅΣ╕¬µ£║σÖ¿Σ║║τÜäΣ╕ïΣ╕Çµ¡Ñ∩╝ë
//   3. σè¿Σ╜£σÉÄσñäτÉå∩╝êσñäτÉåµùïΦ╜¼σè¿Σ╜£πÇüτº╗σè¿µúÇµƒÑ∩╝ë
//
// µêæΣ╗¼τÜäµÄºσê╢σ▒éΣ╕╗ΦªüσüÜ∩╝Ü
// - τ▓╛τí«τÜäµù╢Θù┤τ«íτÉå∩╝êµèèσ░╜σÅ»Φâ╜σñÜτÜäµù╢Θù┤τ╗Ö default planner∩╝ë
// - µ¡úτí«Φ«íτ«ùΘ£ÇΦªüΦºäσêÆτÜäµ¡Ñµò░
//
// Σ╕║Σ╗ÇΣ╣êΣ╕ìµö╣ default planner σåàΘâ¿∩╝ƒ
// - default planner τÜä Flow+PIBT τ╗äσÉêσ╖▓τ╗Åσ╛êσ╝║σñº
// - No Man Sky σåáσå¢ΘÿƒτÜä planner Σ╣ƒµÿ»τ▒╗Σ╝╝τÜäµ₧╢µ₧ä∩╝êσ╝òσ»╝Φ╖»σ╛ä + PIBT∩╝ë
// - τ£ƒµ¡úµïëσ╝Çσ╖«Φ╖¥τÜäµÿ» scheduler τÜäΦ┤¿ΘçÅ∩╝êµêæΣ╗¼σ╖▓τ╗Åσ£¿ TaskScheduler Σ╕¡σñºσ╣àµö╣Φ┐¢∩╝ë
void MAPFPlanner::plan(int time_limit, Plan & plan) 
{
    // τ▓╛τí«Φ«íτ«ùσë⌐Σ╜Öµù╢Θù┤
    int elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - env->plan_start_time).count();
    int limit = time_limit - elapsed_us / 1000 
              - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;
    if (limit < 5) limit = 5;

    // Φ«íτ«ùµ£¼µ¼íΘ£ÇΦªüΦºäσêÆσñÜσ░æµ¡Ñ
    // σà¼σ╝Å∩╝Ümin_planner_communication_time / (action_time * max_counter) + 1
    // 
    // Σ╛ïσ¡É∩╝ÜplanCommTime=1000ms, action_time=1ms, max_counter=10
    //   ΓåÆ 1000 / (1 * 10) + 1 = 101 µ¡Ñ
    //
    // σÉ½Σ╣ë∩╝ÜΣ╕ñµ¼í planning Φ░âτö¿Σ╣ïΘù┤µ£ë 1000ms∩╝îµ»Åµ¡ÑµëºΦíîΘ£ÇΦªü 10 Σ╕¬ tick∩╝ê1ms/tick∩╝ë∩╝î
    //        µëÇΣ╗ÑΘ£ÇΦªü 100 µ¡Ñσè¿Σ╜£µ¥ÑΦªåτ¢ûΦ┐Öµ«╡µù╢Θù┤∩╝î+1 µÿ»σ«ëσà¿Σ╜ÖΘçÅ
    //
    // Σ╕║Σ╗ÇΣ╣êΣ╕ìτ╗Öµ¢┤σñÜµ¡Ñ∩╝ƒ
    // - µ¢┤σñÜµ¡Ñ = µ¢┤Θò┐τÜäΦºäσêÆµù╢Θù┤∩╝îσÅ»Φâ╜Φ╢àµù╢
    // - µ¢┤σ░æµ¡Ñ = σè¿Σ╜£Σ╕ìσñƒ∩╝îµ£║σÖ¿Σ║║σê░Σ╕ïΦ╜« planning σëìΣ╝ÜσÄƒσ£░τ¡ëσ╛à
    // - Φ┐ÖΣ╕¬σà¼σ╝ÅσêÜσÑ╜σ╣│ΦííΣ╕ñΦÇà
    // R167: 20% more steps to reduce waiting
    int min_plan_steps = std::max(1, (int)(env->min_planner_communication_time 
                       / (env->action_time * env->max_counter) * 1.2));

    DefaultPlanner::plan(limit, plan.actions, env, min_plan_steps);
}
