"""Matched initial-error sensitivity experiment on the lemniscate replay."""
from pathlib import Path
import json
import numpy as np
from scipy.spatial.transform import Rotation
from .publication_runner import (nominal_scenario, load_or_run, connection_pair_protocol,
                                connection_pair_passed, connection_test, source_fingerprint,
                                DIAGNOSTICS, DEFINITIONS)
from .publication import write_json, write_csv, connection_realization_row, reaching_summary
from .paper_metrics import compute_persistent_reaching_time, _pose_errors
from ..paper.controller import controller
from ..paper.diagnostics import connection_identity
from ..viz.publication_figures import (panels, legend, save, CONNECTION_STYLES,
                                       connection_realization_figures, theory_figures)

OFFSET = 10.0
EPSILON = 1e-4


def sensitivity_scenario(form, duration=20., dt=.002):
    scenario = nominal_scenario(duration, dt)
    desired = scenario['trajectory'](0.)
    scenario['initial']['V'] = desired['V'] + 4 * np.array([.3, -.2, .1, .4, -.2, .3])
    scenario['controller']['coriolis'] = form
    return scenario


def separation(lc, rb):
    if not np.array_equal(lc['t'], rb['t']):
        raise ValueError('Separation requires matching time samples')
    rotations = [run['H'][:, :3, :3] for run in (lc, rb)]
    relative = rotations[0].transpose(0, 2, 1) @ rotations[1]
    # Free couples about each body's origin, expressed in common inertial axes.
    # These are not moments transported to a shared spatial origin.
    world = [np.concatenate([np.einsum('nij,nj->ni', R, run['wrench'][:, :3]),
                             np.einsum('nij,nj->ni', R, run['wrench'][:, 3:])], axis=1)
             for R, run in zip(rotations, (lc, rb))]
    return {'Position separation [m]': np.linalg.norm(rb['H'][:, :3, 3]-lc['H'][:, :3, 3], axis=1),
            'Attitude separation [deg]': np.degrees(Rotation.from_matrix(relative).magnitude()),
            'Force difference [N]': np.linalg.norm(world[1][:, 3:]-world[0][:, 3:], axis=1),
            'Torque difference [N m]': np.linalg.norm(world[1][:, :3]-world[0][:, :3], axis=1)}


def audit(run, scenario):
    metric = np.asarray(scenario['controller'].get('Lambda_s', np.linalg.inv(scenario['controller']['Lambda'])))
    r = np.sqrt(np.einsum('ni,ij,nj->n', run['s'], metric, run['s']))
    observed = compute_persistent_reaching_time(run, EPSILON, metric, final_time=30., time_offset=OFFSET)
    residuals, relative, command_errors, difference = [], [], [], []
    for i in range(len(run['t'])):
        state = {'H': run['H'][i], 'V': run['V'][i]}
        desired = {'H': run['Hdesired'][i], 'V': run['Vdesired'][i], 'Vdot': run['VdotDesired'][i]}
        identity = connection_identity(state, desired, scenario['controller'], scenario['plantPi'])
        residuals.append(identity['residualNorm'])
        difference.append(identity['differenceNorm'])
        if identity['differenceNorm'] > 1e-6:
            relative.append(identity['residualNorm']/identity['differenceNorm'])
        command = controller(state, desired, scenario['controller'], scenario['plantPi'])[0]
        command_errors.append(float(np.linalg.norm(command-run['wrench'][i])))
    return {'T_obs_elapsed_s': observed, 'T_obs_source_s': None if observed is None else OFFSET+observed,
            'persistent_reaching_elapsed_s': observed,
            'persistent_reaching_absolute_s': None if observed is None else OFFSET+observed,
            'persistent_threshold': EPSILON, 'persistent_final_source_time_s': 30.,
            'persistent_invariance_passed': observed is not None,
            'initial_s': run['s'][0], 'initial_weighted_s': float(r[0]),
            'identity_max_residual': float(max(residuals)),
            'identity_rms_residual': float(np.sqrt(np.mean(np.square(residuals)))),
            'identity_max_relative_residual': max(relative) if relative else None,
            'identity_passed': bool(np.all(np.array(residuals) <= 1e-12+1e-10*np.array(difference))),
            'maximum_command_replay_error': max(command_errors),
            'command_replay_passed': max(command_errors) < 1e-9}


def run_study(root, raw_root):
    root, raw_root = Path(root), Path(raw_root)/'connection-sensitivity'
    metadata = root/'metadata'
    metadata.mkdir(parents=True, exist_ok=True)
    runs, scenarios, audits = {}, {}, {}
    for form in ('lc', 'rb'):
        print(f'Running sensitivity C_{form.upper()} (fresh, 10-30 s)', flush=True)
        scenario = sensitivity_scenario(form)
        run, failure, _ = load_or_run(raw_root/form, scenario, reuse_cache=False)
        if failure:
            write_json(metadata/'connection_sensitivity_failure.json', {'form': form, 'failure': failure})
            raise RuntimeError(f'Sensitivity simulation failed: {failure}')
        runs[form], scenarios[form] = run, scenario
        audits[form] = audit(run, scenario)
    protocol = connection_pair_protocol(runs['lc'], runs['rb'])
    end = 20.
    series = separation(runs['lc'], runs['rb'])
    peaks = {name: {'maximum': float(values.max()), 'absolute_time_s': float(OFFSET+runs['lc']['t'][values.argmax()])}
             for name, values in series.items()}
    fine, fine_audits = {}, {}
    for form in ('lc', 'rb'):
        print(f'Running full-horizon refinement C_{form.upper()} through {OFFSET+end:g} s', flush=True)
        scenario = sensitivity_scenario(form, end, .001)
        fine[form], failure, _ = load_or_run(raw_root/'refinement'/form, scenario, reuse_cache=False)
        if failure:
            write_json(metadata/'connection_sensitivity_failure.json', {'form': form, 'refinement': True, 'failure': failure})
            raise RuntimeError(str(failure))
        fine_audits[form] = audit(fine[form], scenario)
    fine_series = separation(fine['lc'], fine['rb'])
    mask = runs['lc']['t'] <= end
    refinement = {}
    for name, values in series.items():
        aligned = np.interp(runs['lc']['t'][mask], fine['lc']['t'], fine_series[name])
        error = float(np.max(np.abs(values[mask]-aligned)))
        peak = float(np.max(values[mask]))
        refinement[name] = {'coarse_peak_full_horizon': peak, 'fine_peak_full_horizon': float(fine_series[name].max()),
                            'max_trace_difference': error, 'relative_to_coarse_peak': error/peak if peak else None}
    report = {'experiment': '4x imposed initial twist error; no physical payload release',
              'multiplier': 4, 'time_offset_s': OFFSET, 'source_interval_s': [OFFSET, OFFSET+end],
              'persistent_threshold': EPSILON, 'persistent_final_source_time_s': 30.,
              'protocol': protocol, 'protocol_passed': connection_pair_passed(protocol),
              'controller': scenarios['lc']['controller'], 'inertia_parameters': scenarios['lc']['plantPi'],
              'coarse_dt_s': .002, 'fine_dt_s': .001, 'coarse': audits, 'fine': fine_audits,
              'full_run_separation_peaks': peaks, 'refinement': refinement}
    report['qualifications'] = [
        'Persistent reaching is the first saved sample after which the weighted norm remains <=1e-4 through source time 30 s.',
        'Two integration steps measure sensitivity, not established numerical convergence.',
        'LC and RB reaching times are assessed independently and may differ.',
        'Different configurations at reaching may retain position separation while following the same reduced vector field.',
    ]
    write_json(metadata/'connection_sensitivity_protocol.json', report)
    rows = []
    for form in runs:
        row = connection_realization_row(runs[form], form, scenarios[form])
        row.update({'T_obs [source s]': audits[form]['T_obs_source_s'],
                    'Persistent invariance through 30 s': audits[form]['persistent_invariance_passed'],
                    'Identity maximum residual': audits[form]['identity_max_residual']})
        for name, peak in peaks.items():
            row['Pair peak '+name] = peak['maximum']
            row['Pair peak time '+name+' [s]'] = peak['absolute_time_s']
        rows.append(row)
    write_csv(root/'tables'/'connection_sensitivity_summary.csv', rows)
    out = root/'figures'/'04-nominal-validation'
    # Every existing nominal filename is exported from this same sensitivity pair.
    connection_realization_figures(runs, scenarios, root/'figures')
    nominal = reaching_summary(runs['lc'], scenarios['lc'], EPSILON)
    refined_nominal = reaching_summary(fine['lc'], sensitivity_scenario('lc', end, .001), EPSILON)
    nominal.update(experiment='connection-sensitivity', time_offset_s=OFFSET,
                   T_obs_source_s=audits['lc']['T_obs_source_s'],
                   T_obs_source_by_connection={form: audits[form]['T_obs_source_s'] for form in audits},
                   persistent_invariance_passed=audits['lc']['persistent_invariance_passed'],
                   step_refinement={'T_obs_fine': refined_nominal['T_obs'],
                                    'fine_passed': refined_nominal['passed'],
                                    'fine_energy_residual_relative_to_initial': refined_nominal['energy_residual_relative_to_initial']})
    identity = connection_test(runs['lc'], scenarios['lc'])
    theory_figures(runs, scenarios, root/'figures', nominal, identity)
    write_json(metadata/'finite_time_reaching_summary.json', nominal)
    write_json(metadata/'connection_equivalence_summary.json', {
        **{k:v for k,v in identity.items() if k not in ('t', 'difference', 'theory', 'residual')},
        'experiment': 'connection-sensitivity', 'all_sample_audit': audits['lc']})
    write_csv(root/'tables'/'connection_realization_summary.csv',
              [connection_realization_row(runs[k], k, scenarios[k]) for k in runs])
    write_csv(root/'tables'/'nominal_reaching_summary.csv', [{k:nominal[k] for k in (
        'V_s(0)', 'lambda_min(Lambda_s)', 'lambda_max(I)', 'k_d', 'k_s', 'alpha', 'q',
        'c_Lambda', 'a', 'b', 'epsilon_s', 'T_obs', 'T_bound', 'passed')}])
    fig, axes = panels([r'$\|p-p_d\|$ [m]', 'Geodesic attitude\nerror [deg]', r'$\|s\|_{\Lambda_s}$'], False, (OFFSET, 30.))
    for form, run in runs.items():
        pos, angle = _pose_errors(run)
        metric = np.asarray(scenarios[form]['controller'].get('Lambda_s', np.linalg.inv(scenarios[form]['controller']['Lambda'])))
        r = np.sqrt(np.einsum('ni,ij,nj->n', run['s'], metric, run['s']))
        for ax, values in zip(axes, (pos, np.degrees(angle), r)):
            ax.plot(OFFSET+run['t'][mask], values[mask], label=f'$C_{{{form.upper()}}}$', **CONNECTION_STYLES[form])
    axes[-1].set_yscale('symlog', linthresh=EPSILON)
    axes[-1].axhline(EPSILON, color='black', ls=':', lw=.7)
    mark_reaching(axes, audits)
    legend(fig, axes[0]); save(fig, out, 'connection_sensitivity_tracking')
    fig, axes = panels(['Position separation\n[m]', 'Attitude separation\n[deg]',
                        'Force difference\n[N]', 'Torque difference\n[N m]'], False, (OFFSET, 30.))
    for ax, values in zip(axes, series.values()):
        ax.plot(OFFSET+runs['lc']['t'][mask], values[mask], color='black')
    mark_reaching(axes, audits)
    legend(fig, axes[0]); save(fig, out, 'connection_sensitivity_separation')
    notes = ('# Connection sensitivity metrics and diagnostics\n\n'
             'Full run: lemniscate source time 10-30 s. Initial twist perturbation is 4x the original; '
             'pose starts at the desired pose. This is not a physical payload-drop experiment.\n\n'
             'RMSE is sqrt(mean(squared error norm)); attitude uses geodesic degrees. '
              'Persistent reaching is the first sample with weighted s <= 1e-4 that stays below the threshold '
              'through source time 30 s. LC and RB reaching times are assessed independently.\n\n'
             'Separation compares positions in metres, relative rotation angle in degrees, and force/torque '
             'commands rotated into inertial axes. Torque is the commanded free couple about each body origin; '
             'it excludes position-cross-force moments about a common spatial origin. '
             'Independent-trajectory command differences are not the same-state connection identity. '
             'The identity and command replay checks evaluate every logged sample. Identity norms scale '
             'torque by 1 N m and force by 1 N; relative residuals exclude signals <=1e-6.\n\n'
             'Refinement compares 2 ms and 1 ms integration with identical gains through the full interval. '
             'JSON reports trace differences and reaching times without assuming agreement. '
             'Full-run peaks and times may lie outside the focused view. The CSV repeats pair separation '
             'peaks for both controller rows; they are not per-controller performance measures.\n\n')
    notes += '\n'.join(f'- {form}: {audits[form]}' for form in audits)
    notes += '\n\nRefinement relative trace differences: '+str({k:v['relative_to_coarse_peak'] for k,v in refinement.items()})+'\n'
    notes += '\n\n'+'\n'.join('- '+item for item in report['qualifications'])+'\n'
    (metadata/'connection_sensitivity_diagnostics.md').write_text(notes, encoding='utf-8')
    description = ('## Nominal figures: connection sensitivity experiment\n\n'
                   'All figures in figures/04-nominal-validation use the same 4x sensitivity pair. '
                   'The previous 1x experiment is superseded in this folder. '
                   'Initial pose equals the desired lemniscate pose at source time 10 s; '
                   'initial twist error is [1.2,-0.8,0.4] rad/s and [1.6,-0.8,1.2] m/s. '
                   'The plant has exact bare-vehicle inertia, no adaptation, and no physical payload release. '
                   'LC/RB gains are identical. Full tracking and error figures cover source time 10-30 s. '
                   'All time-history figures cover source time 10-30 s. Persistent reaching requires '
                   'weighted s <= 1e-4 through source time 30 s. Each connection has its own reaching time. '
                   'The conservative bound is also an elapsed duration.'\
                   '\n\nSee connection_sensitivity_diagnostics.md for physical separation definitions, '
                   'refinement sensitivity and persistent-invariance checks. The two-step comparison does not '
                   'establish numerical convergence.\n')
    manifest_path = metadata/'manifest.json'
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    status = manifest.get('report', {})
    status.update(nominal_experiment='connection-sensitivity', nominal_replay_duration_s=20.,
                  nominal_reaching_passed=nominal['passed'], nominal_refinement_passed=refined_nominal['passed'],
                  nominal_energy_residual_relative_to_initial=nominal['energy_residual_relative_to_initial'],
                  connection_identity_passed=all(a['identity_passed'] for a in audits.values()),
                  connection_realization_protocol_passed=report['protocol_passed'],
                  nominal_persistent_invariance_passed=all(a['persistent_invariance_passed'] for a in audits.values()))
    manifest['report'] = status
    manifest['connection_sensitivity'] = {'protocol_passed': report['protocol_passed'],
                                          'source_sha256': source_fingerprint()}
    write_json(manifest_path, manifest)
    (metadata/'diagnostic_report.md').write_text(DIAGNOSTICS+'\n'+description+'\n## Current checks\n\n'+
        '\n'.join(f'- {k}: {v}' for k,v in status.items())+'\n', encoding='utf-8')
    (metadata/'metric_definitions.md').write_text(DEFINITIONS+'\n'+description, encoding='utf-8')
    figure_description = description.replace('connection_sensitivity_protocol.json',
        '../../metadata/connection_sensitivity_protocol.json').replace(
        'connection_sensitivity_diagnostics.md', '../../metadata/connection_sensitivity_diagnostics.md')
    (out/'README.md').write_text('# Nominal connection sensitivity figures\n\n'+figure_description, encoding='utf-8')
    write_json(metadata/'connection_sensitivity_manifest.json', {
        'experiment': 'connection-sensitivity', 'export_source_sha256': source_fingerprint(),
        'raw_provenance': {k:runs[k]['metadata'].get('cache') for k in runs},
        'full_source_interval_s': [10.,30.], 'figure_source_interval_s': [10.,30.],
        'figures': sorted(p.name for p in out.iterdir() if p.suffix in ('.pdf','.png'))})
    print(f'Sensitivity artifacts saved; full interval {OFFSET:g} to {OFFSET+end:g} s', flush=True)
    return report


def mark_reaching(axes, audits):
    for form, result in audits.items():
        if result['T_obs_source_s'] is not None:
            for ax in axes:
                ax.axvline(result['T_obs_source_s'], color=CONNECTION_STYLES[form]['color'],
                           ls=':', lw=.8, label=rf'$T_{{\mathrm{{obs}},{form.upper()}}}$')
    axes[-1].set_xlabel('Lemniscate time [s]')
