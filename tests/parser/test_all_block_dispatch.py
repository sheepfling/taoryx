from taoryx.language.problem_parser import parse_problem_text


def test_all_documented_block_keywords_dispatch() -> None:
    text = """(all-blocks)
*title all blocks
*atmos standard
*earth wgs-84
*define global
*egs out.dbf time
*file out.dat time
*optimize a for vel=max on segment 1, trajectory 1
*print time
*radar 1
*search 1
*summarize sum
*survey 1 survey
*units/fmt
*wind geodetic
*trajectory 1 vehicle start on 1
*define local
*dwn/crs long=0
*file trj.dat time
*iip beta=1
*initial geodetic
*print time
*tangent long=0
*segment 1 only
*aero ca=(ca)
*constants g=1
*cg cg=0
*fly alpha=0
*increment wt=-1
*inertial geodetic
*integ dt=0.1
*limits alpha<10
*prop thrust=1
*rail launch
*reset wt=1
*when time>1 stop
*end
"""
    document = parse_problem_text(text)
    assert not [item for item in document.diagnostics if item.severity == "error"]
    problem = document.problems[0]
    assert len(problem.blocks) == 14
    assert len(problem.trajectories[0].blocks) == 7
    assert len(problem.trajectories[0].segments[0].blocks) == 12
####
